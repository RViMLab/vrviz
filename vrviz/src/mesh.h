/*

	Copyright 2011 Etay Meiri

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MESH_H
#define	MESH_H

#include <map>
#include <vector>
#include <GL/glew.h>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>       // Output data structure
#include <assimp/postprocess.h> // Post processing flags
#include "shared/Matrices.h"
#include "texture.h"
#include "openvr.h"

#define SAFE_DELETE(p) if (p) { delete p; p = NULL; }
#define ASSIMP_LOAD_FLAGS (aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices)
#define INVALID_OGL_VALUE 0xffffffff


namespace vr
{
/** A single vertex in a render model */
struct RenderModel_Vertex_t_rgb
{
    HmdVector3_t vPosition;		// position in meters in device space
    HmdVector3_t vNormal;
    HmdVector3_t vColor;
};
}

struct Vertex
{
    Vector3 m_pos;
    Vector2 m_tex;
    Vector3 m_normal;

    Vertex() {}

    Vertex(const Vector3& pos, const Vector2& tex, const Vector3& normal)
    {
        m_pos    = pos;
        m_tex    = tex;
        m_normal = normal;
    }
};


class Mesh
{
public:
    Mesh();

    ~Mesh();

    bool LoadMesh(const std::string& Filename);
    void InitSphere(float radius, Vector3 color, Vector4 center, int num_lat=8, int num_lon=0 );

    void Render();

    std::string name;
    int id;
    std::string frame_id;
    bool has_texture;
    bool initialized;
    float radius;
    Vector3 color;
    Vector4 offset;
    std::string fallback_texture_filename;
    Vector3 scale;
    Matrix4 trans;
    bool Z_UP;

private:
    bool InitFromScene(const aiScene* pScene, const std::string& Filename);
    Vector4 sphere2cart(float azimuth, float elevation, float radius);
    void AddColorVertex(Vector4 pt,Vector4 normal,Vector3 color, std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices);
    void InitMesh(unsigned int Index, const aiMesh* paiMesh, const aiNode* node);
    bool InitMaterials(const aiScene* pScene, const std::string& Filename);
    void Clear();

#define INVALID_MATERIAL 0xFFFFFFFF
#define NO_TEXTURE 0xFFFFFFFE

public:
    struct MeshEntry {
        MeshEntry();

        ~MeshEntry();

        void Init(const std::vector<vr::RenderModel_Vertex_t>& Vertices,
                  const std::vector<u_int32_t>& Indices);
        void Init(const std::vector<vr::RenderModel_Vertex_t_rgb>& Vertices,
                  const std::vector<u_int32_t>& Indices);

        GLuint VB;
        GLuint VA;
        GLuint IB;
        unsigned int NumIndices;
        unsigned int MaterialIndex;
    };

    std::vector<MeshEntry> m_Entries;
    std::vector<Texture*> m_Textures;
};


#endif	/* MESH_H */

