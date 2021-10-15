/*!
 * \file datomsGlBlock.h
 * \brief deformable atoms gl
 * \date 28/01/2018
 * \author Benoît Piranda
 */

#ifndef DATOMSGLBLOCK_H_
#define DATOMSGLBLOCK_H_
#include <string>
#include <objLoader.h>
#include "matrix44.h"
#include "glBlock.h"

namespace Datoms {
class DatomsGlBlock:public GlBlock {
protected :
public :
    Matrix mat{};
    GLuint currentModel;

    DatomsGlBlock(bID id) : GlBlock(id) { currentModel=1; };
    virtual ~DatomsGlBlock() {};

    void glDraw(ObjLoader::ObjLoader *ptrObj) override;
};
}
#endif /* DATOMSGLBLOCK_H_ */
