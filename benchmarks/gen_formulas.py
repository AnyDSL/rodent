#! /usr/bin/python3
import sys

def main():
    formulas = {}
    with open(sys.argv[1], "r") as f:
        for line in f:
            elems = line.split(':')
            ray = elems[1].strip()
            ref = float(elems[-2].strip())
            ours = float(elems[-1].strip())
            if not ray in formulas:
                formulas[ray] = [(ref, ours)]
            else:
                formulas[ray].append((ref, ours))

    for ray, factors in formulas.items():
        print(ray + " = pow(", end="")
        for i, factor in enumerate(factors):
            ref, ours = factor
            print("({}/{})".format(ours, ref), end="")
            if i != len(factors)-1:
                print(" * ",end="")
        print(", 1.0/{})".format(float(len(factors))))                

if __name__ == "__main__":
    main()
