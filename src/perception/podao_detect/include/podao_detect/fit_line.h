#ifndef __FIT_LINE_H__
#define __FIT_LINE_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>

class SegLine {
public:
    int maxIterations_;
    double threshold_;

    SegLine() {
        maxIterations_ = 100;
        threshold_ = 0.01;
    }

    void setMaxIterations(int maxIterations) {
        maxIterations_ = maxIterations;
    }

    void setThreshold(double threshold) {
        threshold_ = threshold;
    }

    // 计算点到直线的距离
    double pointToLineDistance(const pcl::PointXYZ& p, const pcl::PointXYZ& p0, const pcl::PointXYZ& d) {
        double px = p.x - p0.x;
        double py = p.y - p0.y;
        double pz = p.z - p0.z;

        double cross_x = py * d.z - pz * d.y;
        double cross_y = pz * d.x - px * d.z;
        double cross_z = px * d.y - py * d.x;
        double cross_norm = std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
        double d_norm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) + 1e-6; // 避免除零

        return cross_norm / d_norm;
    }

    // **RANSAC 直线拟合**
    void fitLineRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr& inlierIndices, 
        pcl::ModelCoefficients::Ptr& coefficients) {

        int bestInlierCount = 0;
        std::srand(std::time(0)); // 随机数种子

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
        kdtree->setInputCloud(cloud);

        for (int iter = 0; iter < maxIterations_; iter++) {
            // **1. 随机选取两个点**
            int index1 = std::rand() % cloud->size();
            // 搜索当前点的邻域
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;
            kdtree->radiusSearch(cloud->points[index1], 0.2, neighbor_indices, neighbor_distances);
            // 检查邻域点是否满足 Z 方向距离条件
            if (neighbor_indices.size() < 3) continue;

            int index2 =  neighbor_indices[std::rand() % neighbor_indices.size()];
            while (index1 == index2 ) {
                // std::cout << "index1 == index2" << std::endl;
                index2 = neighbor_indices[std::rand() % neighbor_indices.size()];
            }

            pcl::PointXYZ p0 = (*cloud)[index1];
            pcl::PointXYZ p1 = (*cloud)[index2];

            // **2. 计算方向向量 d = p1 - p0**
            pcl::PointXYZ d;
            d.x = p1.x - p0.x;
            d.y = p1.y - p0.y;
            d.z = p1.z - p0.z;

            // 归一化方向向量
            double norm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            if (norm < 1e-6) continue;  // 避免除零
            d.x /= norm;
            d.y /= norm;
            d.z /= norm;

            // **3. 计算内点**
            std::vector<int> inliers;
            for (int i = 0; i < cloud->size(); i++) {
                double dist = pointToLineDistance((*cloud)[i], p0, d);
                if (dist < threshold_) {
                    inliers.push_back(i);
                }
            }

            // **4. 更新最佳直线**
            if (inliers.size() > bestInlierCount) {
                bestInlierCount = inliers.size();
                inlierIndices->indices = inliers;

                optimizeLineDirection(cloud, inlierIndices, coefficients);
            }
        }
    }

    // **最小二乘法优化直线方向**
    void optimizeLineDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr& inlierIndices, 
        pcl::ModelCoefficients::Ptr& coefficients) {
        if (inlierIndices->indices.empty()) return;

        // **1. 计算均值点 P0**
        Eigen::Vector3d meanP(0, 0, 0);
        for (int idx : inlierIndices->indices) {
            meanP += Eigen::Vector3d((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);
        }
        meanP /= inlierIndices->indices.size();

        // **2. 计算协方差矩阵**
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (int idx : inlierIndices->indices) {
            Eigen::Vector3d p((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);
            Eigen::Vector3d diff = p - meanP;
            covariance += diff * diff.transpose();
        }
        covariance /= inlierIndices->indices.size();

        // **3. PCA 分解，找到最大特征值对应的特征向量**
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        Eigen::Vector3d direction = solver.eigenvectors().col(2); // 取最大特征值对应的特征向量

        // **4. 更新直线参数**
        coefficients->values.resize(6);
        coefficients->values[0] = meanP.x();  // P0.x
        coefficients->values[1] = meanP.y();  // P0.y
        coefficients->values[2] = meanP.z();  // P0.z
        coefficients->values[3] = direction.x();  // D.x
        coefficients->values[4] = direction.y();  // D.y
        coefficients->values[5] = direction.z();  // D.z
    }

};

#endif // __FIT_LINE_H__