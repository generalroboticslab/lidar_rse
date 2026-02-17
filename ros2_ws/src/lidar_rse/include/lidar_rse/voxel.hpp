#ifndef VOXEL_H
#define VOXEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/memory.h>   

struct VoxelKey
{
    int64_t x, y, z;
    bool operator==(const VoxelKey& o) const noexcept { return x==o.x && y==o.y && z==o.z;}
};

struct VoxelKeyHash
// the weird numbers are golden-ratio-based constants used to mix bits properly in a hash function so your voxel hash map stays fast and collision-resistant.
{
    std::size_t operator()(VoxelKey const& k) const noexcept
    {
        uint64_t a = static_cast<uint64_t>(k.x);
        uint64_t b = static_cast<uint64_t>(k.y);
        uint64_t c = static_cast<uint64_t>(k.z);
        a += 0x9e3779b97f4a7c15ULL + (b<<6) + (b>>2);
        b += 0x9e3779b97f4a7c15ULL + (c<<6) + (c>>2);
        c += 0x9e3779b97f4a7c15ULL + (a<<6) + (a>>2);
        return static_cast<std::size_t>(a ^ b ^ c);
    }
};

class voxel_map
{
    public:
        using MapT = std::unordered_map<VoxelKey, float, VoxelKeyHash>;

        explicit voxel_map(float leaf_size_m = 0.1f, size_t reserve_hint = 0)
        : leaf(leaf_size_m), inv_leaf(1.0f / std::max(1e-9f, leaf_size_m))
        {
            if (reserve_hint) 
                map.reserve(reserve_hint);
        }    

        inline VoxelKey key_from_point(float x, float y, float z) const noexcept 
        {
            return VoxelKey{
                static_cast<int64_t>(std::floor(x * inv_leaf)),
                static_cast<int64_t>(std::floor(y * inv_leaf)),
                static_cast<int64_t>(std::floor(z * inv_leaf))
            };
        }

        inline void center_from_key(const VoxelKey &k, float &cx, float &cy, float &cz) const noexcept 
        {
            cx = (static_cast<float>(k.x) + 0.5f) * leaf;
            cy = (static_cast<float>(k.y) + 0.5f) * leaf;
            cz = (static_cast<float>(k.z) + 0.5f) * leaf;
        }

        inline void add_log_odds(
            const VoxelKey &k, 
            float delta, 
            float clamp_min = -100.0f, 
            float clamp_max = 100.0f
        ) 
        {
            float &v = map[k];
            v += delta;
            if (v < clamp_min) 
                v = clamp_min;
            else if (v > clamp_max) 
                v = clamp_max;
        }

        inline void set_log_odds(const VoxelKey &k, float value) 
        {
            map[k] = value;
        }

        inline bool get_log_odds(const VoxelKey &k, float &out) const 
        {
            auto it = map.find(k);
            if (it == map.end()) return false;
            out = it->second; return true;
        }

        // this is the main function that we are aiming for
        // basically, if the log-likelihood is lower than a threshold, then we deem it a dynamic pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr extract_dynamic_pcl(float threshold_L) const 
        {
            auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            cloud->points.reserve(map.size()); // upper bound
            for (const auto &kv : map) 
            {
                if (kv.second <= threshold_L) {
                    pcl::PointXYZ p;
                    float cx, cy, cz;
                    center_from_key(kv.first, cx, cy, cz);
                    p.x = cx; p.y = cy; p.z = cz;
                    cloud->points.push_back(p);
                }
            }
            cloud->width  = static_cast<uint32_t>(cloud->points.size());
            cloud->height = 1;
            cloud->is_dense = true;
            return cloud;
        }

        ~voxel_map(){};
        MapT map;

    private:
        
        float leaf;
        float inv_leaf;
};

#endif