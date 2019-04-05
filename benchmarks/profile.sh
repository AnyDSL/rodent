echo "Profiling..."

echo "" > profile.txt

ITER=5

cd renderers

cd living_room
taskset 0x1 bin/rodent --bench $ITER --eye -1.8 1 -5 --dir -0.1 0 1 --up 0 1 0 --fov 60 --width 1920 --height 1088 -o render.png 2> /dev/null >> ../../profile.txt
cd ..

cd bathroom
taskset 0x1 bin/rodent --bench $ITER --eye -2.26 15.62 35.23 --dir -22.18 -5.32 -97.36 --up 0 1 0 --fov 60 --width 1920 --height 1088 -o render.png 2> /dev/null >> ../../profile.txt
cd ..

cd bedroom
taskset 0x1 bin/rodent --bench $ITER --eye 3.5 1 3.5 --dir -1 0 -1 --up 0 1 0 --fov 60 --width 1920 --height 1088 -o render.png 2> /dev/null >> ../../profile.txt
cd ..

cd dining_room
taskset 0x1 bin/rodent --bench $ITER --eye -4 1.3 0.0 --dir 1 -0.1 0 --up 0 1 0 --fov 48 --width 1920 --height 1088 -o render.png 2> /dev/null >> ../../profile.txt
cd ..

cd kitchen
taskset 0x1 bin/rodent --bench $ITER --eye 0.5 1.6 3 --dir -0.4 -0.05 -1 --up 0 1 0 --fov 60 --width 1920 --height 1088 -o render.png 2> /dev/null >> ../../profile.txt
cd ..

cd staircase
taskset 0x1 bin/rodent --bench $ITER --eye 0 1.6 4.5 --dir 0 0 -1 --up 0 1 0 --fov 38 --height 1280 --width 720 -o render.png 2> /dev/null >> ../../profile.txt
cd ..
