#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = splitMethod == SplitMethod::NAIVE ? recursiveBuild(primitives) : recursiveBuildWithSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildWithSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    const auto costRayBoxIntersection = 0.125f;
    const auto costRayPrimitiveIntersection = 1.0f;

    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildWithSAH(std::vector{objects[0]});
        node->right = recursiveBuildWithSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // 包围所有 primitive 中心点的 bounding box
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        int dim = centroidBounds.maxExtent();

        // 1. init bucket info for sah partition buckets
        const int N_BUCKET = 12;
        struct SAHBucket {
            int cnt;
            Bounds3 bounds;
        } buckets[N_BUCKET];
        for (int i = 0; i < objects.size(); i++) {
            auto t = N_BUCKET * centroidBounds.Offset(objects[i]->getBounds().Centroid());
            int j = dim == 0 ? t.x : dim == 1 ? t.y : t.z;
            if (j == N_BUCKET) j--;
            buckets[j].bounds = Union(buckets[j].bounds, objects[i]->getBounds());
        }

        // 2. calc cost for splitting after each bucket and find minimal
        float cost[N_BUCKET - 1];
        float minCost = std::numeric_limits<float>().max();
        int minCostSplitBucketIdx = 0;
        for (int i = 0; i < N_BUCKET - 1; i++) {
            Bounds3 b0, b1;
            int cnt0 = 0, cnt1 = 0;
            for (int j = 0; j <= i; j++) {
                b0 = Union(b0, buckets[j].bounds);
                cnt0 += buckets[j].cnt;
            }
            for (int j = i + 1; j < N_BUCKET; j++) {
                b1 = Union(b1, buckets[j].bounds);
                cnt1 += buckets[j].cnt;
            }
            cost[i] =
                costRayBoxIntersection
                + (cnt0 * costRayPrimitiveIntersection * b0.SurfaceArea() / bounds.SurfaceArea())
                + (cnt1 * costRayPrimitiveIntersection * b1.SurfaceArea() / bounds.SurfaceArea());
            if (cost[i] < minCost) {
                minCost = cost[i];
                minCostSplitBucketIdx = i;
            }
        }

        // 3. either create leaf or split primitives at selected sah bucket
        // 在作业框架里, node 只能存一个 primitive, 所以只要 n > 2 就 split
        // float leafCost = objects.size();
        // if (objects.size() > maxPrimsInNode || minCost < leafCost) {
            // split
        // }

        auto beginning = objects.begin();
        auto ending = objects.end();
        auto middling = std::partition(objects.begin(), objects.end(), [=](Object* obj) {
            auto t = N_BUCKET * centroidBounds.Offset(obj->getBounds().Centroid());
            int bucket_idx = dim == 0 ? t.x : dim == 1 ? t.y : t.z;
            if (bucket_idx == N_BUCKET) bucket_idx--;
            return bucket_idx <= minCostSplitBucketIdx;
        });

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        // assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildWithSAH(leftshapes);
        node->right = recursiveBuildWithSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    auto hasIntersection = node->bounds.IntersectP(ray);
    if (!hasIntersection) {
        return {};
    }

    auto isLeaf = node->left == nullptr && node->right == nullptr;
    if (isLeaf) {
        return node->object->getIntersection(ray);
    }

    // return the closer one
    auto leftIntersection = getIntersection(node->left, ray);
    auto rightIntersection = getIntersection(node->right, ray);
    return leftIntersection.distance < rightIntersection.distance
        ? leftIntersection
        : rightIntersection;
}