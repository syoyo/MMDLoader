newoption {
   trigger = "with-glm",
   description = "Build with GLM replacement for gluPerspecive and gluLookat"
}

sources = {
   "viewer_main.cc",
   "trackball.cpp"
   }

mmd_sources = {
   "mmd_scene.cc",
   "pmd_reader.cc",
   "vmd_reader.cc",
   "vmd_animation.cc",
   }

-- premake4.lua
solution "MMDTestSolution"
   configurations { "Release", "Debug" }

   if (os.is("windows")) then
      platforms { "x32", "x64" }
   else
      platforms { "native", "x32", "x64" }
   end

   -- A project defines one build target
   project "MMDTest"
      kind "ConsoleApp"
      language "C++"
      files { sources, mmd_sources }

      includedirs {
         "./"
      }

      -- MacOSX. Guess we use gcc.
      configuration { "macosx", "gmake" }
         defines { '_LARGEFILE_SOURCE', '_FILE_OFFSET_BITS=64' }

         links { "OpenGL.framework", "GLUT.framework" }  -- Use system's SDL

         -- Bullet physics
         -- defines { 'ENABLE_BULLET' }
         -- includedirs { "./../../extlibs/bullet/bullet-2.79/src" }
         -- libdirs { "./../../extlibs/bullet/bullet-2.79/src/BulletDynamics"
         --         , "./../../extlibs/bullet/bullet-2.79/src/BulletCollision" 
         --         , "./../../extlibs/bullet/bullet-2.79/src/LinearMath" } 
         -- links { "BulletDynamics", "BulletCollision", "LinearMath" }

      configuration { "macosx", "xcode4" }
         includedirs {
            "/Library/Frameworks/SDL.framework/Headers"
         }
         
      -- Windows specific
      configuration { "windows" }
         defines { 'NOMINMAX', '_LARGEFILE_SOURCE', '_FILE_OFFSET_BITS=64' }

      -- Linux specific
      configuration { "linux", "gmake" }
         defines { '_LARGEFILE_SOURCE', '_FILE_OFFSET_BITS=64' }
         if _OPTIONS["with-glm"] then
            defines { 'ENABLE_GLM' }
         end
         -- defines { 'ENABLE_BULLET' }
         -- includedirs { "./extlibs/bullet/bullet3/src" }
         -- libdirs { "./extlibs/bullet/bullet3/src/BulletDynamics"
         --         , "./extlibs/bullet/bullet3/src/BulletCollision" 
         --         , "./extlibs/bullet/bullet3/src/LinearMath" } 
         -- links { "BulletDynamics", "BulletCollision", "LinearMath" }
         links { "GL", "glut" }

      configuration "Debug"
         defines { "DEBUG" } -- -DDEBUG
         flags { "Symbols" }
         targetname "mmdview_debug"

      configuration "Release"
         -- defines { "NDEBUG" } -- -NDEBUG
         flags { "Symbols", "Optimize" }
         targetname "mmdview"
