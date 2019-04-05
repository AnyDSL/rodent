#! /usr/bin/python3
import subprocess

iters = "20"
warmups = "5"
bench_rodent = "../build_embree2/bin/bench_traversal"
bench_embree = "../build_embree2/bin/bench_embree"
variants = [
    "--ray-width 8 --bvh-width 4",
    "--ray-width 8 --bvh-width 4 -p",
    "--bvh-width 4 -s",
    "--ray-width 8 --bvh-width 8",
    "--ray-width 8 --bvh-width 8 -p",
    "--bvh-width 8 -s"
]
scenes = [
    "sponza", 
    "crown",
    "san-miguel",
    "powerplant"
]
offsets = {
    "sponza":     (0.01, 10.0),
    "crown":      (0.01, 10.0),
    "san-miguel": (0.01, 5.0),
    "powerplant": (0.01, 1000.0)
}

def bench_mrays(args):
    pipe = subprocess.Popen(args, stdout = subprocess.PIPE)  
    for line in pipe.stdout:
        elems = line.split()
        if elems[1] == b'Mrays/sec':
            return float(elems[0])
    return None

def main():
    distribs = ["primary", "ao", "bounces"]
    for scene in scenes:
        for variant in variants:
            for rays in distribs:
                (tmin, ao_max) = offsets[scene]
                tmax = 1.0e9
                args = ["-ray", "scenes/" + scene + "/" + rays + ".rays", "--bench", iters, "--warmup", warmups]
                if rays == "ao":
                    tmax = ao_max
                    args += ["-any"]
                args += ["--tmin", str(tmin), "--tmax", str(tmax)]
                args += variant.split()
                #print(scene, ": ", " ".join(args))
                mrays_embree = bench_mrays([bench_embree, "-obj", "scenes/" + scene + "/" + scene + ".obj"] + args) if not "-p" in variant else None
                mrays_rodent = bench_mrays([bench_rodent, "-bvh", "scenes/" + scene + "/" + scene + ".bvh"] + args)
                print("{} : {} : {} : {} : {}".format(scene, rays, variant, mrays_embree, mrays_rodent))

if __name__ == "__main__":
    main()
