#!/usr/bin/python3
import sys

def print_results(results, rays):
    for ray, ray_name in rays:
        elem = next((res for res in results if res[1] == ray), None)
        if elem == None:
            print("& -- & -- ", end="")
        else:
            ref = float(elem[-2]) if elem[-2] != 'None' else None
            res = float(elem[-1]) if elem[-1] != 'None' else None
            if ref == None or res == None:
                if res == None:
                    print("& -- ", end="")
                else:
                    print("& {:0.2f} ".format(res), end="")
                if ref == None:
                    print("& -- ", end="")
                else:
                    print("& {:0.2f} ".format(ref), end="")
            else:
                print("& {:0.2f} ({:+0.0f}\\%) ".format(res, 100.0*(res - ref) / ref), end = "")
                print("& {:0.2f} ".format(ref), end = "")


def main():
    rays = [
        ("primary", "Primary"),
        ("ao", "AO"),
        ("bounces", "Diffuse")
    ]
    tables = [
        {
            "title"    : "BVH2",
            "variants" : []
        },
        {
            "title"    : "BVH4",
            "variants" : [
                ("--bvh-width 4 -s", "Single"),
                ("--ray-width 8 --bvh-width 4 -p", "Packet"),
                ("--ray-width 8 --bvh-width 4",    "Hybrid")
            ]
        },
        {
            "title"    : "BVH8",
            "variants" : [
                ("--bvh-width 8 -s", "Single"),
                ("--ray-width 8 --bvh-width 8 -p", "Packet"),
                ("--ray-width 8 --bvh-width 8",    "Hybrid")
            ]
        }
    ]
    scenes = [
        ("sponza", "Sponza"),
        ("crown", "Crown"),
        ("san-miguel", "San-Miguel"),
        ("powerplant", "Powerplant")
    ]
    results = []

    for line in sys.stdin.readlines():
        elems = line.split(":")
        scene = elems[0].strip()
        ray = elems[1].strip()
        variant = elems[2].strip()
        ref = elems[-2].strip()
        res = elems[-1].strip()
        results.append((scene, ray, variant, ref, res))

    for table in tables:
        title = table["title"]
        variants = table["variants"]
        print("% {}".format(title))
        for scene, scene_name in scenes:
            if len(variants) > 0:
                print("\midrule")
                print("\\multirow{{{}}}{{*}}{{{}}} & ".format(len(variants), scene_name), end="")
            else:
                print("{} ".format(scene_name), end="")
            kept_scene = list(filter(lambda x: x[0] == scene, results))
            if len(variants) > 0:
                for i, (variant, variant_name) in enumerate(variants):
                    if i > 0:
                        print(" & ", end="")
                    print("{} ".format(variant_name), end="")
                    kept_variant = filter(lambda x: x[2] == variant, kept_scene)
                    print_results(kept_variant, rays)
                    print("\\\\")
            else:
                print_results(kept_scene, rays)
                print("\\\\")

if __name__ == "__main__":
    main()
