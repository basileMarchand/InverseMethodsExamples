import os
env=Environment( ENV=os.environ )
# Mandatory flags and defines
env.Append(LINKFLAGS=["-lm","-lpthread","-lrt", "-ldl", "-fopenmp", "-msse2"])
env.Append(CCFLAGS =['-std=c++11', '-fPIC', '-fopenmp', "-msse2"])
env.Append(PREFIX=os.path.join("#","lib"))
env.Append(CPPPATH="#/include")

conf_file = "cache.py"
vars = Variables(conf_file, ARGUMENTS)
vars.Add(BoolVariable("DEBUG", "Set to True to compil in debug mode", False))
vars.Add("EIGEN_INC", "Path to the Eigen header file", "/usr/include")
vars.Update( env )

if env["DEBUG"]:
   env.Append(CCFLAGS=["-ggdb3", "-O0"])
else:
   env.Append(CCFLAGS=["-ggdb3", "-O3"])

env.Append(CPPPATH=[env["EIGEN_INC"],])
src_files=Glob('src/*.cpp', strings=True)

### Build the static library
lib = env.StaticLibrary("InverseExamples", source=src_files)
install_lib = env.Install( Dir("#/lib") , lib )
env.Append(LIBPATH=Dir("#/lib") )

### Build the two examples
# 1 - Kalman filter example
kalman_src = "examples/main_kalman.cpp"
prog1 = env.Program( "main_kalman", kalman_src, LIBS=["InverseExamples"])
install_bin1 = env.Install( Dir("#/examples"), prog1 )

# 2 - Determinist
determinist_src = "examples/main_determinist.cpp"
prog2 = env.Program( "main_determinist", determinist_src, LIBS=["InverseExamples"])
install_bin2 = env.Install( Dir("#/examples"), prog2 )

Alias("lib", [lib, install_lib])
Alias("kalman", [prog1, install_bin1])
Alias("determinist", [prog2, install_bin2])
Default( Alias("lib"), Alias("kalman"), Alias("determinist"))


### Save configuration 
vars.Save(conf_file, env )

