#! /usr/bin/python3
import subprocess
import os

iters = "500"
warmups = "100"
bin_dir = "/space/perard/sources/rodent/build_embree2/bin"
bench_dir = "/space/perard/sources/rodent/benchmarks"
bench_rodent = "./bench_traversal"
bench_aila = "./bench_aila"
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
    pipe = subprocess.Popen(args, stdout = subprocess.PIPE, env=dict(os.environ, ANYDSL_PROFILE='full'), cwd=bin_dir)
    for line in pipe.stdout:
        elems = line.split()
        if elems[1] == b'Mrays/sec':
            return float(elems[0])
    return None

def main():
    distribs = ["primary", "ao", "bounces"]
    for scene in scenes:
        for rays in distribs:
            (tmin, ao_max) = offsets[scene]
            tmax = 1.0e9
            if rays == "ao":
                tmax = ao_max
            args = ["-ray", bench_dir + "/scenes/" + scene + "/" + rays + ".rays", "--tmin", str(tmin), "--tmax", str(tmax), "--bench", iters, "--warmup", warmups]
            #print(scene, ": ", " ".join(args))
            mrays_aila = bench_mrays([bench_aila, "-bvh", bench_dir + "/scenes/" + scene + "/" + scene + ".bvh"] + args)
            mrays_rodent = bench_mrays([bench_rodent, "-gpu", "nvvm", "-bvh", bench_dir + "/scenes/" + scene + "/" + scene + ".bvh"] + args)
            print("{} : {} : {} : {}".format(scene, rays, mrays_aila, mrays_rodent))

if __name__ == "__main__":
    main()
