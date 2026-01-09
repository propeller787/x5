
// Copyright 2018 Baidu. Inc All Rights Reserved
// Author: Fuxiangyu (fuxiangyu@baidu.com)
// Date: 2018-06-19 14:44
//
// Description: aaboxkdtree for 3d element

#ifndef _MATH_COMMON_AABOXKDTREE3D_H_
#define _MATH_COMMON_AABOXKDTREE3D_H_

#pragma once

#include "math/aabox3d.h"
#include "math/math_utils.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace decision::math {

struct AABox3dKDTreeParams {
    // The maximum depth of the kdtree.
    int max_depth = -1;
    // The maximum number of items in one leaf node.
    int max_leaf_size = -1;
    // The maximum dimension size of leaf node.
    double max_leaf_dimension = -1.0;
};

template <class ObjectType>
class AABoxKDTree3dNode {
public:
    using ObjectPtr = const ObjectType*;

    AABoxKDTree3dNode(const std::vector<ObjectPtr>& objects,
        const AABox3dKDTreeParams& params, int depth)
        : _depth(depth)
    {
        CHECK(!objects.empty());

        compute_boundary(objects);
        compute_partition();

        if (split_to_subnodes(objects, params)) {
            std::vector<ObjectPtr> left_subnode_objects;
            std::vector<ObjectPtr> right_subnode_objects;
            partition_objects(objects, &left_subnode_objects, &right_subnode_objects);

            // Split to sub-nodes.
            if (!left_subnode_objects.empty()) {
                _left_subnode.reset(new AABoxKDTree3dNode<ObjectType>(
                    left_subnode_objects, params, depth + 1));
            }
            if (!right_subnode_objects.empty()) {
                _right_subnode.reset(new AABoxKDTree3dNode<ObjectType>(
                    right_subnode_objects, params, depth + 1));
            }
        } else {
            init_objects(objects);
        }
    }

    ObjectPtr get_nearest_object(const Vec3d& point) const
    {
        ObjectPtr nearest_object = nullptr;
        double min_distance_sqr = std::numeric_limits<double>::infinity();
        get_nearest_object_internal(point, &min_distance_sqr, &nearest_object);
        return nearest_object;
    }

    std::vector<ObjectPtr> get_objects(const Vec3d& point, const double distance) const
    {
        std::vector<ObjectPtr> result_objects;
        get_objects_internal(point, distance, sqr(distance), &result_objects);
        return result_objects;
    }

    AABox3d get_bounding_box() const
    {
        return AABox3d({ _min_x, _min_y, _min_z }, { _max_x, _max_y, _max_z });
    }

private:
    void init_objects(const std::vector<ObjectPtr>& objects)
    {
        _num_objects = objects.size();
        _objects_sorted_by_min = objects;
        _objects_sorted_by_max = objects;
        std::sort(_objects_sorted_by_min.begin(), _objects_sorted_by_min.end(),
            [&](ObjectPtr obj1, ObjectPtr obj2) {
                return _partition == PARTITION_X ? obj1->aabox().min_x() < obj2->aabox().min_x()
                                                 : (_partition == PARTITION_Y ? obj1->aabox().min_y() < obj2->aabox().min_y()
                                                                              : obj1->aabox().min_z() < obj2->aabox().min_z());
            });
        std::sort(_objects_sorted_by_max.begin(), _objects_sorted_by_max.end(),
            [&](ObjectPtr obj1, ObjectPtr obj2) {
                return _partition == PARTITION_X ? obj1->aabox().max_x() > obj2->aabox().max_x()
                                                 : (_partition == PARTITION_Y ? obj1->aabox().max_y() > obj2->aabox().max_y()
                                                                              : obj1->aabox().max_z() > obj2->aabox().max_z());
            });
        _objects_sorted_by_min_bound.reserve(_num_objects);
        for (ObjectPtr object : _objects_sorted_by_min) {
            _objects_sorted_by_min_bound.push_back(
                _partition == PARTITION_X ? object->aabox().min_x()
                                          : (_partition == PARTITION_Y ? object->aabox().min_y()
                                                                       : object->aabox().min_z()));
        }
        _objects_sorted_by_max_bound.reserve(_num_objects);
        for (ObjectPtr object : _objects_sorted_by_max) {
            _objects_sorted_by_max_bound.push_back(
                _partition == PARTITION_X ? object->aabox().max_x()
                                          : (_partition == PARTITION_Y ? object->aabox().max_y()
                                                                       : object->aabox().max_z()));
        }
    }

    bool split_to_subnodes(const std::vector<ObjectPtr>& objects, const AABox3dKDTreeParams& params)
    {
        if (params.max_depth >= 0 && _depth >= params.max_depth) {
            return false;
        }
        if (static_cast<int>(objects.size()) <= std::max(1, params.max_leaf_size)) {
            return false;
        }
        double max_dim = std::max(_max_x - _min_x, _max_y - _min_y);
        max_dim = std::max(max_dim, _max_z - _min_z);
        if (params.max_leaf_dimension >= 0.0 && max_dim <= params.max_leaf_dimension) {
            return false;
        }
        return true;
    }

    double lowerbound_distance_sqr_to_point(const Vec3d& point) const
    {
        double dx = 0.0;
        if (point.x() < _min_x) {
            dx = _min_x - point.x();
        } else if (point.x() > _max_x) {
            dx = point.x() - _max_x;
        }
        double dy = 0.0;
        if (point.y() < _min_y) {
            dy = _min_y - point.y();
        } else if (point.y() > _max_y) {
            dy = point.y() - _max_y;
        }
        double dz = 0.0;
        if (point.z() < _min_z) {
            dz = _min_z - point.z();
        } else if (point.z() > _max_z) {
            dz = point.z() - _max_z;
        }
        return dx * dx + dy * dy + dz * dz;
    }

    double upperbound_distance_sqr_to_point(const Vec3d& point) const
    {
        const double dx = (point.x() > _mid_x ? (point.x() - _min_x) : (point.x() - _max_x));
        const double dy = (point.y() > _mid_y ? (point.y() - _min_y) : (point.y() - _max_y));
        const double dz = (point.z() > _mid_z ? (point.z() - _min_z) : (point.z() - _max_z));
        return dx * dx + dy * dy + dz * dz;
    }

    void get_all_objects(std::vector<ObjectPtr>* const result_objects) const
    {
        result_objects->insert(result_objects->end(),
            _objects_sorted_by_min.begin(), _objects_sorted_by_min.end());
        if (_left_subnode != nullptr) {
            _left_subnode->get_all_objects(result_objects);
        }
        if (_right_subnode != nullptr) {
            _right_subnode->get_all_objects(result_objects);
        }
    }

    void get_objects_internal(const Vec3d& point,
        const double distance,
        const double distance_sqr,
        std::vector<ObjectPtr>* const result_objects) const
    {
        if (lowerbound_distance_sqr_to_point(point) > distance_sqr) {
            return;
        }
        if (upperbound_distance_sqr_to_point(point) <= distance_sqr) {
            get_all_objects(result_objects);
            return;
        }
        const double pvalue = (_partition == PARTITION_X ? point.x()
                                                         : (_partition == PARTITION_Y ? point.y() : point.z()));
        if (pvalue < _partition_position) {
            const double limit = pvalue + distance;
            for (int i = 0; i < _num_objects; ++i) {
                if (_objects_sorted_by_min_bound[i] > limit) {
                    break;
                }
                ObjectPtr object = _objects_sorted_by_min[i];
                if (object->distance_sqr_to(point) <= distance_sqr) {
                    result_objects->push_back(object);
                }
            }
        } else {
            const double limit = pvalue - distance;
            for (int i = 0; i < _num_objects; ++i) {
                if (_objects_sorted_by_max_bound[i] < limit) {
                    break;
                }
                ObjectPtr object = _objects_sorted_by_max[i];
                if (object->distance_sqr_to(point) <= distance_sqr) {
                    result_objects->push_back(object);
                }
            }
        }
        if (_left_subnode != nullptr) {
            _left_subnode->get_objects_internal(point, distance, distance_sqr, result_objects);
        }
        if (_right_subnode != nullptr) {
            _right_subnode->get_objects_internal(point, distance, distance_sqr, result_objects);
        }
    }

    void get_nearest_object_internal(const Vec3d& point,
        double* const min_distance_sqr,
        ObjectPtr* const nearest_object) const
    {
        if (lowerbound_distance_sqr_to_point(point) >= *min_distance_sqr - kMathEpsilon) {
            return;
        }
        const double pvalue = (_partition == PARTITION_X ? point.x()
                                                         : (_partition == PARTITION_Y ? point.y() : point.z()));
        const bool search_left_first = (pvalue < _partition_position);
        if (search_left_first) {
            if (_left_subnode != nullptr) {
                _left_subnode->get_nearest_object_internal(
                    point, min_distance_sqr, nearest_object);
            }
        } else {
            if (_right_subnode != nullptr) {
                _right_subnode->get_nearest_object_internal(
                    point, min_distance_sqr, nearest_object);
            }
        }
        if (*min_distance_sqr <= kMathEpsilon) {
            return;
        }

        if (search_left_first) {
            for (int i = 0; i < _num_objects; ++i) {
                const double bound = _objects_sorted_by_min_bound[i];
                if (bound > pvalue && sqr(bound - pvalue) > *min_distance_sqr) {
                    break;
                }
                ObjectPtr object = _objects_sorted_by_min[i];
                const double distance_sqr = object->distance_sqr_to(point);
                if (distance_sqr < *min_distance_sqr) {
                    *min_distance_sqr = distance_sqr;
                    *nearest_object = object;
                }
            }
        } else {
            for (int i = 0; i < _num_objects; ++i) {
                const double bound = _objects_sorted_by_max_bound[i];
                if (bound < pvalue && sqr(bound - pvalue) > *min_distance_sqr) {
                    break;
                }
                ObjectPtr object = _objects_sorted_by_max[i];
                const double distance_sqr = object->distance_sqr_to(point);
                if (distance_sqr < *min_distance_sqr) {
                    *min_distance_sqr = distance_sqr;
                    *nearest_object = object;
                }
            }
        }
        if (*min_distance_sqr <= kMathEpsilon) {
            return;
        }
        if (search_left_first) {
            if (_right_subnode != nullptr) {
                _right_subnode->get_nearest_object_internal(
                    point, min_distance_sqr, nearest_object);
            }
        } else {
            if (_left_subnode != nullptr) {
                _left_subnode->get_nearest_object_internal(
                    point, min_distance_sqr, nearest_object);
            }
        }
    }

    void compute_boundary(const std::vector<ObjectPtr>& objects)
    {
        _min_x = std::numeric_limits<double>::infinity();
        _min_y = std::numeric_limits<double>::infinity();
        _min_z = std::numeric_limits<double>::infinity();
        _max_x = -std::numeric_limits<double>::infinity();
        _max_y = -std::numeric_limits<double>::infinity();
        _max_z = -std::numeric_limits<double>::infinity();
        for (ObjectPtr object : objects) {
            _min_x = std::min(_min_x, object->aabox().min_x());
            _max_x = std::max(_max_x, object->aabox().max_x());
            _min_y = std::min(_min_y, object->aabox().min_y());
            _max_y = std::max(_max_y, object->aabox().max_y());
            _min_z = std::min(_min_z, object->aabox().min_z());
            _max_z = std::max(_max_z, object->aabox().max_z());
        }
        _mid_x = (_min_x + _max_x) / 2.0;
        _mid_y = (_min_y + _max_y) / 2.0;
        _mid_z = (_min_z + _max_z) / 2.0;
    }

    void compute_partition()
    {
        double dx = _max_x - _min_x;
        double dy = _max_y - _min_y;
        double dz = _max_z - _min_z;
        if (dx >= dy && dx >= dz) {
            _partition = PARTITION_X;
            _partition_position = (_min_x + _max_x) / 2.0;
        } else if (dy >= dx && dy >= dz) {
            _partition = PARTITION_Y;
            _partition_position = (_min_y + _max_y) / 2.0;
        } else {
            _partition = PARTITION_Z;
            _partition_position = (_min_z + _max_z) / 2.0;
        }
    }

    void partition_objects(const std::vector<ObjectPtr>& objects,
        std::vector<ObjectPtr>* const left_subnode_objects,
        std::vector<ObjectPtr>* const right_subnode_objects)
    {
        left_subnode_objects->clear();
        right_subnode_objects->clear();
        std::vector<ObjectPtr> other_objects;
        if (_partition == PARTITION_X) {
            for (ObjectPtr object : objects) {
                if (object->aabox().max_x() <= _partition_position) {
                    left_subnode_objects->push_back(object);
                } else if (object->aabox().min_x() >= _partition_position) {
                    right_subnode_objects->push_back(object);
                } else {
                    other_objects.push_back(object);
                }
            }
        } else if (_partition == PARTITION_Y) {
            for (ObjectPtr object : objects) {
                if (object->aabox().max_y() <= _partition_position) {
                    left_subnode_objects->push_back(object);
                } else if (object->aabox().min_y() >= _partition_position) {
                    right_subnode_objects->push_back(object);
                } else {
                    other_objects.push_back(object);
                }
            }
        } else {
            for (ObjectPtr object : objects) {
                if (object->aabox().max_z() <= _partition_position) {
                    left_subnode_objects->push_back(object);
                } else if (object->aabox().min_z() >= _partition_position) {
                    right_subnode_objects->push_back(object);
                } else {
                    other_objects.push_back(object);
                }
            }
        }
        init_objects(other_objects);
    }

private:
    int _num_objects = 0;
    std::vector<ObjectPtr> _objects_sorted_by_min;
    std::vector<ObjectPtr> _objects_sorted_by_max;
    std::vector<double> _objects_sorted_by_min_bound;
    std::vector<double> _objects_sorted_by_max_bound;
    int _depth = 0;

    // Boundary
    double _min_x = 0.0;
    double _max_x = 0.0;
    double _min_y = 0.0;
    double _max_y = 0.0;
    double _min_z = 0.0;
    double _max_z = 0.0;
    double _mid_x = 0.0;
    double _mid_y = 0.0;
    double _mid_z = 0.0;

    enum Partition {
        PARTITION_X = 1,
        PARTITION_Y = 2,
        PARTITION_Z = 3,
    };
    Partition _partition = PARTITION_X;
    double _partition_position = 0.0;

    std::unique_ptr<AABoxKDTree3dNode<ObjectType>> _left_subnode = nullptr;
    std::unique_ptr<AABoxKDTree3dNode<ObjectType>> _right_subnode = nullptr;
};

template <class ObjectType>
class AABoxKDTree3d {
public:
    using ObjectPtr = const ObjectType*;

    AABoxKDTree3d(const std::vector<ObjectType>& objects, const AABox3dKDTreeParams& params)
    {
        if (!objects.empty()) {
            std::vector<ObjectPtr> object_ptrs;
            for (const auto& object : objects) {
                object_ptrs.push_back(&object);
            }
            _root.reset(new AABoxKDTree3dNode<ObjectType>(object_ptrs, params, 0));
        }
    }

    ObjectPtr get_nearest_object(const Vec3d& point) const
    {
        return _root == nullptr ? nullptr : _root->get_nearest_object(point);
    }

    std::vector<ObjectPtr> get_objects(const Vec3d& point, const double distance) const
    {
        if (_root == nullptr) {
            return {};
        }
        return _root->get_objects(point, distance);
    }

    AABox3d get_bounding_box() const
    {
        return _root == nullptr ? AABox3d() : _root->get_bounding_box();
    }

private:
    std::unique_ptr<AABoxKDTree3dNode<ObjectType>> _root = nullptr;
};

}

#endif // _MATH_COMMON_AABOXKDTREE3D_H_
