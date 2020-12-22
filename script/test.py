import os
import platform
import argparse

os_name = platform.system()

num_threads = [1, 2, 4, 6, 8, 10, 12]


def make(args):
    if (args.mak):
        os.system("mkdir -p ../bin")
        os.system("cd ../src && make")
        if (os_name == "Darwin"):
            os.system("cd ../test && make baseline && make")
        elif (os_name == "Linux"):
            os.system("cd ../test && make")


def run(args):
    if (os_name == "Darwin"):
        if (args.baseline):
            print("baseline:")
            print("test3 with ", num_threads, " threads")
            for t in num_threads:
                os.system("../bin/test3_baseline " + str(t) + " 1000 100000")
            # print("test5")
            # os.system("../bin/test5_baseline")
        print("pmalloc:")
        print("test3 with", num_threads, "threads")
        for t in num_threads:
            os.system("../bin/test3 " + str(t) + " 1000 100000")
        # print("test5")
        # os.system("../bin/test5")
    elif (os_name == "Linux"):
        if (args.baseline):
            print("baseline:")
            for t in num_threads:
                os.system("../bin/test3_baseline " + str(t) + " 1000 100000")
            print("Hoard:")
            for t in num_threads:
                os.system(
                    "LD_PRELOAD=$HOME/Hoard/src/libhoard.so ../bin/test3 " + str(t) + " 1000 100000")
            print("TCmalloc:")
            for t in num_threads:
                os.system(
                    "LD_PRELOAD=$HOME/tools/lib/libtcmalloc.so ../bin/test3 " + str(t) + " 1000 100000")
            # print("test5")
            # os.system("../bin/test5")
        print("pmalloc:")
        print("test3 with", num_threads, " threads")
        for t in num_threads:
            os.system(
                "LD_PRELOAD=$HOME/pmalloc/bin/libhoard.so ../bin/test3 " + str(t) + " 1000 100000")
        # print("test5")
        # os.system("LD_PRELOAD=../bin/libhoard.so ../bin/test5")


def main():
    parser = argparse.ArgumentParser(description='PMALLOC test')
    parser.add_argument("-m", dest="mak", action="store_true",
                        default=False, help="make lib and tests")
    parser.add_argument("-b", dest="baseline", action="store_true",
                        default=False, help="show baseline results")

    args = parser.parse_args()

    print(args)
    make(args)
    run(args)


if __name__ == '__main__':
    main()
