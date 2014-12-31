g++ -O3 soil/image_DXT.c soil/image_helper.c soil/stb_image_aug.c soil/SOIL.c tex_bmp.cc pmd_reader.cc vmd_reader.cc mmd_scene.cc trackball.cpp vmd_animation.cc viewer_main.cc -lglut -lGL -lGLU -lGLEW -lglfw -Wwrite-strings
./a.out $1/Normal.pmd queer_motion.vmd $1/
#./a.out hzeo_kaitoV3formal/hzeoV3kaito_F.pmx test/queer_motion.vmd

