#ifndef GO_UNDERGROUND_DBSCAN_H
#define GO_UNDERGROUND_DBSCAN_H

#include <pcl/pcl_base.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <queue>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2


inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANSimpleCluster {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
	
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < input_cloud_->points.size(); i++) {
            if (types[i] == PROCESSED) {
                continue;
            }
            int nn_size = radiusSearch(i, eps_, nn_indices, nn_distances);
            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }
protected:
    PointCloudPtr input_cloud_;
    double eps_ {0.0};
    int minPts_ {1}; // not including the point itself.
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};
    KdTreePtr search_method_;

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }
}; // class DBSCANCluster



template <typename PointT>
void extractClusterFromSeedPoint(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    const PointT& seed_point,
    double radius,
    int max_points,
    std::vector<int>& cluster_indices)
{
    cluster_indices.clear();
    if (!cloud || cloud->empty()) return;

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    /* ---------- Step 1: 找最近的 seed index ---------- */
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    if (kdtree.nearestKSearch(seed_point, 1, nn_indices, nn_dists) == 0)
        return;

    int seed_idx = nn_indices[0];

    /* ---------- Step 2: 半径连通 BFS ---------- */
    std::vector<bool> visited(cloud->size(), false);
    std::queue<int> q;

    visited[seed_idx] = true;
    q.push(seed_idx);
    cluster_indices.push_back(seed_idx);

    std::vector<int> neighbors;
    std::vector<float> distances;

    while (!q.empty())
    {
        int idx = q.front();
        q.pop();

        neighbors.clear();
        distances.clear();

        if (kdtree.radiusSearch(
                cloud->points[idx],
                radius,
                neighbors,
                distances) > 0)
        {
            for (int nbr_idx : neighbors)
            {
                if (!visited[nbr_idx])
                {
                    visited[nbr_idx] = true;
                    q.push(nbr_idx);
                    cluster_indices.push_back(nbr_idx);

                    if (max_points > 0 &&
                        cluster_indices.size() >= max_points)
                        return;
                }
            }
        }
    }
}


#endif