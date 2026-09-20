* make build directory in their respective experiments:
    
    $mkdir build

* go to build directory:
    
    $cd build

* run cmake to generate cmakefiles:
    
    $cmake ..

* run make to actaully build the cpp files (j4 is to make it use all the cores so that it builds fast)

    $ make -j4

* to run the output build files in the build directory run:
    
    $ ./name_of_the_output_file