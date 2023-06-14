//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // 随机数 * 总发光表面积, 用来随机选取光源进行采样
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto inter = Scene::intersect(ray);
    if (!inter.happened) {
        return Scene::backgroundColor;
    }

    // indir light
    Vector3f dir_light, indir_light;
    
    auto p = inter.coords;
    auto n = inter.normal;
    auto m = inter.m;
    auto wo = -ray.direction;
 
    // pdf light = 1 / sum of emit object surface area
    Intersection sample_light_inter;
    float pdf_light;
    sampleLight(sample_light_inter, pdf_light);

    auto x = sample_light_inter.coords;
    auto nn = sample_light_inter.normal;
    auto emit = sample_light_inter.emit;
    auto ws = x - p;

    auto r = Ray(p, ws);
    auto light_inter = Scene::intersect(r);

    // sample light not block in middle
    if (light_inter.happened && fabs(sample_light_inter.distance - light_inter.distance) < 0.00000001) {
        dir_light =
            emit
            * m->eval(wo, ws, n)
            * dotProduct(ws, n) // ws 和交点法线夹角
            * dotProduct(ws, nn) // ws 和光源法线夹角
            / std::pow(((x - p).norm()), 2)
            / pdf_light;
    }


    if (get_random_float() < RussianRoulette) {
        auto wi = m->sample(wo, n);
        auto pdf_obj = m->pdf(wo, wi, n);

        auto r = Ray(p, wi);
        auto obj_inter = Scene::intersect(r);

        // if ray hit a non-emitting object
        if (obj_inter.happened && !obj_inter.obj->hasEmit()) {
            indir_light =
                castRay(r, depth + 1)
                * m->eval(wo, wi, n)
                * dotProduct(wi, n)
                / pdf_obj
                / RussianRoulette;
                
        }
    }

    return dir_light + indir_light;
}