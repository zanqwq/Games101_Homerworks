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

Vector3f shade(Intersection inter, Vector3f wo) {
    if (!inter.happened) {
        return Scene::backgroundColor;
    }

    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    // indir light
    Vector3f dir_light, indir_light;
    
    auto p = inter.coords;
    auto n = inter.normal;
    auto m = inter.m;
 
    // pdf light = 1 / sum of emit object surface area
    Intersection sample_light_inter;
    float pdf_light;
    sampleLight(sample_light_inter, pdf_light);

    auto x = sample_light_inter.coords;
    auto nn = sample_light_inter.normal;
    auto emit = sample_light_inter.emit;
    auto ws = (x - p).normalized();

    auto light_inter = Scene::intersect(Ray(p, ws));
    // sample light not block in middle
    if (light_inter.happened && fabs(sample_light_inter.distance - light_inter.distance) < 0.00000001) {
        auto f_r = m->eval(ws, wo, n);
        auto cosA = std::max(.0f, dotProduct(ws, n));
        auto cosB = std::max(.0f, dotProduct(-ws, nn));
        auto r2 = std::pow(((x - p).norm()), 2);

        dir_light =
            emit
            * f_r
            * cosA // ws 和交点法线夹角
            * cosB // ws 和光源法线夹角
            / r2
            / pdf_light;
    }


    if (get_random_float() < RussianRoulette) {
        auto wi = m->sample(wo, n).normalized();
        auto pdf_obj = m->pdf(wi, wo, n);
 
        auto obj_inter = Scene::intersect(Ray(p, wi));
        // if ray hit a non-emitting object
        if (obj_inter.happened && !obj_inter.obj->hasEmit()) {
            // TODO:
            auto f_r = m->eval(wi, wo, n);
            auto cosA = std::max(.0f, dotProduct(wi, n));
            indir_light =
                // TODO
                shade(obj_inter, wi)
                * f_r
                * cosA
                / pdf_obj
                / Scene::RussianRoulette;
                
        }
    }

    return dir_light + indir_light;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    return shade(Scene::intersect(ray), -ray.direction);
}