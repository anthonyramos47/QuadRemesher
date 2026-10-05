# Third-party notices

QuadRemesher is distributed under the GNU General Public License v3 (see
`LICENSE`). Its source is at https://github.com/anthonyramos47/QuadRemesher.
The binaries include the following third-party components.

| component | license | source |
|---|---|---|
| libQEx (quad extraction) | GPL-3.0 | https://github.com/hcebke/libQEx |
| CoMISo (MIQ solver), via libigl `copyleft/comiso` | GPL-3.0 | https://www.graphics.rwth-aachen.de/software/comiso/ |
| gmm 4.2 (bundled in CoMISo) | LGPL-2.1+ with GCC Runtime Library Exception | http://getfem.org/gmm |
| libigl v2.5.0 (core, OpenGL/GLFW viewer) | MPL-2.0 | https://github.com/libigl/libigl |
| Eigen | MPL-2.0 | https://gitlab.com/libeigen/eigen |
| OpenMesh 11.0 | BSD-3-Clause | https://www.graphics.rwth-aachen.de/software/openmesh/ |
| GLFW | zlib | https://www.glfw.org |
| glad (generated OpenGL loader) and khrplatform.h | Khronos (MIT-style) | https://github.com/Dav1dde/glad |
| Reference BLAS (Linux build, linked statically) | BSD-style (netlib) | https://www.netlib.org/blas/ |
| OpenBLAS DLLs (Windows build, shipped in `bin/`) | BSD-3-Clause | https://github.com/OpenMathLib/OpenBLAS |
| libgfortran, libquadmath, libgcc DLLs (Windows build), libstdc++/libgcc (Linux build, static) | GPL-3.0 with GCC Runtime Library Exception / LGPL-2.1 (libquadmath) | https://gcc.gnu.org |

The MPL-2.0 components (libigl, Eigen) are used unmodified; their source code
is available at the links above. The full MPL-2.0 text is at
https://mozilla.org/MPL/2.0/.

The license texts of the permissively licensed components follow.

## OpenMesh

```
                              OpenMesh
          Copyright (c) 2001-2015, RWTH-Aachen University
          Department of Computer Graphics and Multimedia
                         All rights reserved.
                           www.openmesh.org

---------------------------------------------------------------------------
This file is part of OpenMesh.
---------------------------------------------------------------------------

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## GLFW

```
Copyright (c) 2002-2006 Marcus Geelnard

Copyright (c) 2006-2019 Camilla Löwy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would
   be appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not
   be misrepresented as being the original software.

3. This notice may not be removed or altered from any source
   distribution.

```

## Khronos (khrplatform.h, used by glad)

```
Copyright (c) 2008-2018 The Khronos Group Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and/or associated documentation files (the
"Materials"), to deal in the Materials without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Materials, and to
permit persons to whom the Materials are furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Materials.

THE MATERIALS ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
```
