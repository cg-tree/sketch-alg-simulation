import os
import subprocess
from docopt import docopt
usage=''' Usage:
make.py [<target>]
make.py [--Include path] [<target>]

-I path --Include path  #include directory to be passed to compile statement
'''

print(os.listdir(".."))
pymac = "c++ -O3 -Wall -shared -std=c++11 -undefined dynamic_lookup $(python3 -m pybind11 --includes) pysketch.cpp -o pysketch$(python3-config --extension-suffix)"

pylinux = "c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) example.cpp -o example$(python3-config --extension-suffix)"

pyinclude = "c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3-config --includes) -Iextern/pybind11/include pysketch.cpp -o pysketch$(python3-config --extension-suffix)"

def make(target):
  
  if target=="pymac":
    args = [pymac]

  elif target == "pylinux":
    args = [pylinux]
  
  elif target == "main":
    args = ["c++", "main.cpp","-O3","-Wall","-std=c++11", "-o", "main"]
  
  else:
    print(f"{target} is not a valid target")
    exit()

  args = ' '.join(args)
  return subprocess.run(args, shell=1, text=1,capture_output=1)

def main():
  cargs = docopt(usage)
  target = "main"
  
  if cargs["<target>"] != None:
    target = cargs["<target>"]
  
  process = make(target)
  print(process.stderr)

if __name__=="__main__":
  main()
