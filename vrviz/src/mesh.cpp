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



#include "mesh.h"

Mesh::MeshEntry::MeshEntry()
{
    VB = INVALID_OGL_VALUE;
    IB = INVALID_OGL_VALUE;
    NumIndices  = 0;
    MaterialIndex = INVALID_MATERIAL;
};

Mesh::MeshEntry::~MeshEntry()
{
    if (VB != INVALID_OGL_VALUE)
    {
        glDeleteBuffers(1, &VB);
    }

    if (IB != INVALID_OGL_VALUE)
    {
        glDeleteBuffers(1, &IB);
    }
}

void Mesh::MeshEntry::Init(const std::vector<vr::RenderModel_Vertex_t>& Vertices,
                          const std::vector<u_int32_t>& Indices)
{
    NumIndices = Indices.size();

    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &VA );
    glBindVertexArray( VA );

    // Populate a vertex buffer
    glGenBuffers( 1, &VB );
    glBindBuffer( GL_ARRAY_BUFFER, VB );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t ) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, rfTextureCoord ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &IB );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, IB );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( u_int32_t ) * NumIndices, &Indices[0], GL_STATIC_DRAW );

    glBindVertexArray( 0 );

}


void Mesh::MeshEntry::Init(const std::vector<vr::RenderModel_Vertex_t_rgb>& Vertices,
                          const std::vector<u_int32_t>& Indices)
{
    NumIndices = Indices.size();

    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &VA );
    glBindVertexArray( VA );

    // Populate a vertex buffer
    glGenBuffers( 1, &VB );
    glBindBuffer( GL_ARRAY_BUFFER, VB );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t_rgb ) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vColor ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &IB );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, IB );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( u_int32_t ) * NumIndices, &Indices[0], GL_STATIC_DRAW );

    glBindVertexArray( 0 );

}


Mesh::Mesh()
{
    trans=Matrix4().identity();
    scale.x=1.0;
    scale.y=1.0;
    scale.z=1.0;
    Z_UP=false;
}


Mesh::~Mesh()
{
    Clear();
}


void Mesh::Clear()
{
    for (unsigned int i = 0 ; i < m_Textures.size() ; i++) {
        SAFE_DELETE(m_Textures[i]);
    }
}


bool Mesh::LoadMesh(const std::string& Filename)
{
    // Release the previously loaded mesh (if it exists)
    Clear();
    
    bool Ret = false;
    Assimp::Importer Importer;

    const aiScene* pScene = Importer.ReadFile(Filename.c_str(), ASSIMP_LOAD_FLAGS);

    if (pScene) {
        Ret = InitFromScene(pScene, Filename);
    }
    else {
        printf("Error parsing '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
    }

    return Ret;
}

bool Mesh::InitFromScene(const aiScene* pScene, const std::string& Filename)
{  
    m_Entries.resize(pScene->mNumMeshes);
    m_Textures.resize(pScene->mNumMaterials);

    InitMaterials(pScene, Filename);

    // Initialize the meshes in the scene one by one
    for (unsigned int i = 0 ; i < m_Entries.size() ; i++) {
        const aiMesh* paiMesh = pScene->mMeshes[i];
        InitMesh(i, paiMesh, pScene->mRootNode);
    }

    return true;
}

void Mesh::InitMesh(unsigned int Index, const aiMesh* paiMesh, const aiNode* node)
{

    // Make sure we have a root node
    if (!node) {
      return;
    }

    // We need to fix the orientation
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode) {
      // Don't convert to y-up orientation, which is what the root node in
      // Assimp does
      if (pnode->mParent != NULL) {
        transform = pnode->mTransformation * transform;
      }
      pnode = pnode->mParent;
    }
    // Get just the rotation, for transforming the normals
    aiMatrix3x3 rotation(transform);




    m_Entries[Index].MaterialIndex = paiMesh->mMaterialIndex;

    std::vector<u_int32_t> Indices;


    bool use_texture = (paiMesh->mMaterialIndex < m_Textures.size() && m_Textures[paiMesh->mMaterialIndex]);

    if(use_texture){
        const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
        std::vector<vr::RenderModel_Vertex_t> Vertices;

        for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
            aiVector3D pos = paiMesh->mVertices[i];
            aiVector3D n = paiMesh->mNormals[i];



            Vector4 pt;
            if(Z_UP){
                pt.x= pos.x*scale.x;
                pt.z= pos.y*scale.y;
                pt.y=-pos.z*scale.z;
            }else{
                pt.x=pos.x*scale.x;
                pt.y=pos.y*scale.y;
                pt.z=pos.z*scale.z;
            }
            pt.w=1;

            Vector4 pt_trans = trans * pt;

            Vector4 nm;
            if(Z_UP){
                nm.x= n.x;
                nm.z= n.y;
                nm.y=-n.z;
            }else{
                nm.x=n.x;
                nm.y=n.y;
                nm.z=n.z;
            }
            nm.w=0; // Normals are vectors in free space, so w=0 keeps it from being affected by the translation part

            Vector4 nm_trans = trans * nm;


            const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
            vr::RenderModel_Vertex_t v;
            v.vPosition.v[0]=pt_trans.x;
            v.vPosition.v[1]=pt_trans.y;
            v.vPosition.v[2]=pt_trans.z;
            v.rfTextureCoord[0]=pTexCoord->x;
            v.rfTextureCoord[1]=pTexCoord->y;
            v.vNormal.v[0]=nm_trans.x;
            v.vNormal.v[1]=nm_trans.y;
            v.vNormal.v[2]=nm_trans.z;


            Vertices.push_back(v);
        }
        for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
            const aiFace& Face = paiMesh->mFaces[i];
            if(Face.mNumIndices != 3){
                std::cout << "numfaces not 3, it is " << Face.mNumIndices << std::endl;
            }else{
                Indices.push_back(Face.mIndices[0]);
                Indices.push_back(Face.mIndices[1]);
                Indices.push_back(Face.mIndices[2]);
            }
        }

        m_Entries[Index].Init(Vertices, Indices);

    }else{
        const aiColor4D Zero4D(0.0f, 0.0f, 0.0f, 0.0f);
        std::vector<vr::RenderModel_Vertex_t_rgb> Vertices;

        for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
            aiVector3D pos = paiMesh->mVertices[i];
            aiVector3D n = paiMesh->mNormals[i];



            Vector4 pt;
            if(Z_UP){
                pt.x= pos.x*scale.x;
                pt.z= pos.y*scale.y;
                pt.y=-pos.z*scale.z;
            }else{
                pt.x=pos.x*scale.x;
                pt.y=pos.y*scale.y;
                pt.z=pos.z*scale.z;
            }
            pt.w=1;

            Vector4 pt_trans = trans * pt;

            Vector4 nm;
            if(Z_UP){
                nm.x= n.x;
                nm.z= n.y;
                nm.y=-n.z;
            }else{
                nm.x=n.x;
                nm.y=n.y;
                nm.z=n.z;
            }
            nm.w=0; // Normals are vectors in free space, so w=0 keeps it from being affected by the translation part

            Vector4 nm_trans = trans * nm;


            const aiColor4D* pVertColor = paiMesh->HasVertexColors(0) ? &(paiMesh->mColors[0][i]) : &Zero4D;
            vr::RenderModel_Vertex_t_rgb v;
            v.vPosition.v[0]=pt_trans.x;
            v.vPosition.v[1]=pt_trans.y;
            v.vPosition.v[2]=pt_trans.z;
            v.vColor.v[0]=pVertColor->r;
            v.vColor.v[1]=pVertColor->g;
            v.vColor.v[2]=pVertColor->b;
            v.vNormal.v[0]=nm_trans.x;
            v.vNormal.v[1]=nm_trans.y;
            v.vNormal.v[2]=nm_trans.z;


            Vertices.push_back(v);
        }
        for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
            const aiFace& Face = paiMesh->mFaces[i];
            if(Face.mNumIndices != 3){
                std::cout << "numfaces not 3, it is " << Face.mNumIndices << std::endl;
            }else{
                Indices.push_back(Face.mIndices[0]);
                Indices.push_back(Face.mIndices[1]);
                Indices.push_back(Face.mIndices[2]);
            }
        }

        m_Entries[Index].Init(Vertices, Indices);
    }

}

bool Mesh::InitMaterials(const aiScene* pScene, const std::string& Filename)
{
    // Extract the directory part from the file name
    std::string::size_type SlashIndex = Filename.find_last_of("/");
    std::string Dir;

    if (SlashIndex == std::string::npos) {
        Dir = ".";
    }
    else if (SlashIndex == 0) {
        Dir = "/";
    }
    else {
        Dir = Filename.substr(0, SlashIndex);
    }

    bool Ret = true;

    // Initialize the materials
    for (unsigned int i = 0 ; i < pScene->mNumMaterials ; i++) {
        const aiMaterial* pMaterial = pScene->mMaterials[i];

        m_Textures[i] = NULL;

        if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
            aiString Path;

            if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
                std::string FullPath = Dir + "/" + Path.data;
                m_Textures[i] = new Texture(GL_TEXTURE_2D, FullPath.c_str());

                if (!m_Textures[i]->Load()) {
                    printf("Error loading texture '%s'\n", FullPath.c_str());
                    delete m_Textures[i];
                    m_Textures[i] = NULL;
                    Ret = false;
                }
                else {
                    printf("Loaded texture '%s'\n", FullPath.c_str());
                }
            }
        }
    }

    return Ret;
}

void Mesh::Render()
{

    for (unsigned int i = 0 ; i < m_Entries.size() ; i++) {

        glBindVertexArray( m_Entries[i].VA );


        const unsigned int MaterialIndex = m_Entries[i].MaterialIndex;

        if (MaterialIndex < m_Textures.size() && m_Textures[MaterialIndex]) {
            m_Textures[MaterialIndex]->Bind(GL_TEXTURE0);
        }

        glDrawElements(GL_TRIANGLES, m_Entries[i].NumIndices, GL_UNSIGNED_INT, 0);
    }

}
