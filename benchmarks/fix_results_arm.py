#!/usr/bin/python3
import sys

def main():
    rays = ["primary", "ao", "bounces"]
    scenes = [
        ("sponza"),
        ("crown"),
        ("san-miguel"),
        ("powerplant")
    ]
    ref_variant = "-w 4 -s"
    results = []

    for line in sys.stdin.readlines():
        elems = line.split(":")
        scene = elems[0].strip()
        ray = elems[1].strip()
        variant = elems[2].strip()
        ref = elems[-2].strip()
        res = elems[-1].strip()
        results.append((scene, ray, variant, ref, res))

    for scene in scenes:
        kept_scene = list(filter(lambda res: res[0] == scene, results))
        for ray in rays:
            res_a, res_b = list(filter(lambda res: res[1] == ray, kept_scene))
            if res_b[2] == ref_variant:
                res_a, res_b = res_b, res_a
            print("{} : {} : {} : {} : {}".format(scene, ray, "fixed", res_a[-1], res_b[-1]))
            

if __name__ == "__main__":
    main()
