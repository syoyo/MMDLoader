#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <float.h>

#include <iostream>
#include <vector>
#include <cmath>

#if defined(__APPLE__) && defined(__MACH__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef ENABLE_BULLET
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#endif

#include <assert.h>
#include "trackball.h"

#include "pmd_reader.h"
#include "vmd_reader.h"
#include "mmd_scene.h"
#include "mmd_math.h"

#include "tex_bmp.h"
#include "soil/SOIL.h"

// image loader
//#include "stb_image.c"

using namespace mmd;

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

/*
 * global variable
 */
static int mouse_x, mouse_y; /* x, y location		*/
static float curr_quat[4];   /* current quaternion state	*/
static float prev_quat[4];   /* previous quaternion state	*/
static int mouse_moving;     /* Is mouse moving?		*/
static int spinning;         /* Is scene spinning?		*/
static int width, height;    /* current window width&height	*/
static float zoom;           /* zoom factor			*/
static int mouse_r_pressed;  /* Is right mouse pressed?	*/
static int mouse_m_pressed;  /* Is middle mouse pressed?	*/
static GLfloat view_org[3];  /* view origin			*/
static GLfloat view_tgt[3];  /* view target			*/
static float bmin[3] = {-1, -1, -1}, bmax[3] = {1, 1, 1};
static float center[3] = {0.0, 0.0, 0.0};
static float maxval;
static float scenesize = 20.0f;
static int current_frame = 0;
static int sub_frame = 0;
static int frame_step = 1; // less = faster playback.

char tex_path[256];

PMDModel *model = NULL;
VMDAnimation *anim = NULL;
MMDScene *scene = NULL;

float *renderVertices = NULL;

static void reshape(int w, int h);
static void animate();

float collist[7][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 1.0, 0.0},
    {1.0, 0.0, 1.0},
    {0.0, 1.0, 1.0},
    {1.0, 1.0, 1.0},
};

static inline void MyQSlerp(Quaternion &p, const Quaternion &q,
                            const Quaternion &r, double t) {
  double qdotr = q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w;
  // Clamp to prevent NaN. But is this OK?
  if (qdotr > 1.0)
    qdotr = 1.0;
  if (qdotr < -1.0)
    qdotr = -1.0;

  if (qdotr < 0.0) {
    double theta = acos(-qdotr);

    if (fabs(theta) < 1.0e-10) {
      p = q;
      return;
    }

    double st = sin(theta);
    double inv_st = 1.0 / st;
    double c0 = sin((1.0 - t) * theta) * inv_st;
    double c1 = sin(t * theta) * inv_st;

    p.x = c0 * q.x - c1 * r.x;
    p.y = c0 * q.y - c1 * r.y;
    p.z = c0 * q.z - c1 * r.z;
    p.w = c0 * q.w - c1 * r.w;

  } else {

    double theta = acos(qdotr);

    if (fabs(theta) < 1.0e-10) {
      p = q;
      return;
    }

    double st = sin(theta);
    double inv_st = 1.0 / st;
    double c0 = sin((1.0 - t) * theta) * inv_st;
    double c1 = sin(t * theta) * inv_st;

    p.x = c0 * q.x + c1 * r.x;
    p.y = c0 * q.y + c1 * r.y;
    p.z = c0 * q.z + c1 * r.z;
    p.w = c0 * q.w + c1 * r.w;
  }
}

#ifdef ENABLE_BULLET
static void InitSimulation() {
  btDefaultCollisionConfiguration *config =
      new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);
  btBroadphaseInterface *broadphase = new btDbvtBroadphase();

  btSequentialImpulseConstraintSolver *seqSol =
      new btSequentialImpulseConstraintSolver();

  btConstraintSolver *solver = seqSol;
}

static void StepSimulation() {}
#endif

static void DumpMatrix(float *m) {
  printf("%f, %f, %f, %f\n", m[0], m[1], m[2], m[3]);
  printf("%f, %f, %f, %f\n", m[4], m[5], m[6], m[7]);
  printf("%f, %f, %f, %f\n", m[8], m[9], m[10], m[11]);
  printf("%f, %f, %f, %f\n", m[12], m[13], m[14], m[15]);
}

static double BezierEval(unsigned char *ip, float t) {
  double xa = ip[0] / 256.0;
  double xb = ip[2] / 256.0;
  double ya = ip[1] / 256.0;
  double yb = ip[3] / 256.0;

  double min = 0;
  double max = 1;

  double ct = t;
  while (true) {
    double x11 = xa * ct;
    double x12 = xa + (xb - xa) * ct;
    double x13 = xb + (1.0 - xb) * ct;

    double x21 = x11 + (x12 - x11) * ct;
    double x22 = x12 + (x13 - x12) * ct;

    double x3 = x21 + (x22 - x21) * ct;

    if (fabs(x3 - t) < 0.0001) {
      double y11 = ya * ct;
      double y12 = ya + (yb - ya) * ct;
      double y13 = yb + (1.0 - yb) * ct;

      double y21 = y11 + (y12 - y11) * ct;
      double y22 = y12 + (y13 - y12) * ct;

      double y3 = y21 + (y22 - y21) * ct;

      return y3;
    } else if (x3 < t) {
      min = ct;
    } else {
      max = ct;
    }
    ct = min * 0.5 + max * 0.5;
  }
}

static void VertexTransform(float *vbuffer) {
  for (int i = 0; i < model->vertices_.size(); i++) {
    PMDVertex &pv = model->vertices_[i];
    Vector3 p0, p1;
    p0.x = pv.pos[0];
    p0.y = pv.pos[1];
    p0.z = pv.pos[2];
    p1.x = pv.pos[0];
    p1.y = pv.pos[1];
    p1.z = pv.pos[2];
    unsigned short b0 = pv.bone[0];
    unsigned short b1 = pv.bone[1];

    float *m0 = model->bones_[b0].matrix;
    float *m1 = model->bones_[b1].matrix;
    // printf("b[%d] = ", b0); DumpMatrix(m0);
    // printf("b[%d] = ", b1); DumpMatrix(m1);

    Vector3 v0;
    Vector3 v1;
    Vector3 v;

    // Bone matrix is defined in absolute coordinate.
    // Pass a vertex in relative coordiate to bone matrix.
    p0.x -= model->bones_[b0].pos[0];
    p0.y -= model->bones_[b0].pos[1];
    p0.z -= model->bones_[b0].pos[2];
    p1.x -= model->bones_[b1].pos[0];
    p1.y -= model->bones_[b1].pos[1];
    p1.z -= model->bones_[b1].pos[2];

    MatVMul(v0, m0, p0);
    MatVMul(v1, m1, p1);
    float w = pv.weight / 100.0f;
    v.x = w * v0.x + (1.0f - w) * v1.x;
    v.y = w * v0.y + (1.0f - w) * v1.y;
    v.z = w * v0.z + (1.0f - w) * v1.z;
    // printf("p[%d] %f, %f, %f\n", i, p.x, p.y, p.z);
    // printf("v[%d] %f, %f, %f\n", i, v.x, v.y, v.z);

    vbuffer[3 * i + 0] = v.x;
    vbuffer[3 * i + 1] = v.y;
    vbuffer[3 * i + 2] = v.z;
    // vbuffer[3*i+0] = p.x;
    // vbuffer[3*i+1] = p.y;
    // vbuffer[3*i+2] = p.z;
  }
}

struct MotionSegment {
  int m0;
  int m1;
};

static MotionSegment FindMotionSegment(int frame,
                                       std::vector<Motion> &motions) {
  MotionSegment ms;
  ms.m0 = 0;
  ms.m1 = motions.size() - 1;

  if (frame >= motions[ms.m1].frameNo) {
    ms.m0 = ms.m1;
    ms.m1 = -1;
    return ms;
  }

  while (true) {
    int middle = (ms.m0 + ms.m1) / 2;

    if (middle == ms.m0) {
      return ms;
    }

    if (motions[middle].frameNo == frame) {
      ms.m0 = middle;
      ms.m1 = -1;
      return ms;
    } else if (motions[middle].frameNo > frame) {
      ms.m1 = middle;
    } else {
      ms.m0 = middle;
    }
  }
}

static void InterpolateMotion(Quaternion &rotation, float position[3],
                              std::vector<Motion> &motions, MotionSegment ms,
                              float frame) {
  if (ms.m1 == -1) {
    position[0] = motions[ms.m0].pos[0];
    position[1] = motions[ms.m0].pos[1];
    position[2] = motions[ms.m0].pos[2];
    rotation.x = motions[ms.m0].rotation[0];
    rotation.y = motions[ms.m0].rotation[1];
    rotation.z = motions[ms.m0].rotation[2];
    rotation.w = motions[ms.m0].rotation[3];
  } else {
    int diff = motions[ms.m1].frameNo - motions[ms.m0].frameNo;
    float a0 = frame - motions[ms.m0].frameNo;
    float ratio = a0 / diff; // [0, 1]

    // Use interpolation parameter
    float tx = BezierEval(motions[ms.m0].interpX, ratio);
    float ty = BezierEval(motions[ms.m0].interpY, ratio);
    float tz = BezierEval(motions[ms.m0].interpZ, ratio);
    float tr = BezierEval(motions[ms.m0].interpR, ratio);
    position[0] =
        (1.0 - tx) * motions[ms.m0].pos[0] + tx * motions[ms.m1].pos[0];
    position[1] =
        (1.0 - ty) * motions[ms.m0].pos[1] + ty * motions[ms.m1].pos[1];
    position[2] =
        (1.0 - tz) * motions[ms.m0].pos[2] + tz * motions[ms.m1].pos[2];
    Quaternion r0;
    r0.x = motions[ms.m0].rotation[0];
    r0.y = motions[ms.m0].rotation[1];
    r0.z = motions[ms.m0].rotation[2];
    r0.w = motions[ms.m0].rotation[3];
    Quaternion r1;
    r1.x = motions[ms.m1].rotation[0];
    r1.y = motions[ms.m1].rotation[1];
    r1.z = motions[ms.m1].rotation[2];
    r1.w = motions[ms.m1].rotation[3];
    MyQSlerp(rotation, r0, r1, tr);

    //// Linear
    // position[0] = (1.0 - t) * motions[ms.m0].pos[0] + t *
    // motions[ms.m1].pos[0];
    // position[1] = (1.0 - t) * motions[ms.m0].pos[1] + t *
    // motions[ms.m1].pos[1];
    // position[2] = (1.0 - t) * motions[ms.m0].pos[2] + t *
    // motions[ms.m1].pos[2];

    // Quaternion r0;
    // r0.x = motions[ms.m0].rotation[0];
    // r0.y = motions[ms.m0].rotation[1];
    // r0.z = motions[ms.m0].rotation[2];
    // r0.w = motions[ms.m0].rotation[3];
    // Quaternion r1;
    // r1.x = motions[ms.m1].rotation[0];
    // r1.y = motions[ms.m1].rotation[1];
    // r1.z = motions[ms.m1].rotation[2];
    // r1.w = motions[ms.m1].rotation[3];
    // MyQSlerp(rotation, r0, r1, t);
  }
}

// DBG
static void DumpBone() {
  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    printf("bone[%d] pos = %f, %f, %f\n", i, b.pos[0], b.pos[1], b.pos[2]);
  }
}

// DBG
static void DumpIK() {
  for (int i = 0; i < model->iks_.size(); i++) {
    IK &ik = model->iks_[i];
    printf("ik[%d] root bone: %d (%d target, %d chains, %d iter, %f weight)\n",
           i, ik.boneIndex, ik.targetBoneIndex, ik.chainLength, ik.iterations,
           ik.weight);
    for (int k = 0; k < ik.childBoneIndices.size(); k++) {
      printf("  child[%d] bone: %d\n", k, ik.childBoneIndices[k]);
      printf("    parent = %d\n",
             model->bones_[ik.childBoneIndices[k]].parentIndex);
    }
  }
}

static void UpdateIK() {
  IKSolver solver;

  //// Clear update flag
  // for (int i = 0; i < model->bones_.size(); i++) {
  //  model->bones_[i].updated = false;
  //}

  for (int i = 0; i < model->iks_.size(); i++) {
    IK &ik = model->iks_[i];
    solver.Solve(model, &ik, 0.001);
  }

  //// Clear update flag
  // for (int i = 0; i < model->bones_.size(); i++) {
  //  model->bones_[i].updated = false;
  //}
}

// Recursive.
static void UpdateBoneMatrix(Bone &bone) {
  if (bone.updated == false) {
    if (bone.parentIndex == 0xFFFF) {
      for (int i = 0; i < 16; i++) {
        bone.matrix[i] = bone.matrixTemp[i];
      }
    } else {
      Bone &parent = model->bones_[bone.parentIndex];
      UpdateBoneMatrix(parent);
      MatMul(bone.matrix, parent.matrix, bone.matrixTemp);
    }
    bone.updated = true;
  }
}

static void SetBoneMatrix(int idx, Bone &bone, int frame) {
  bone.matrixTemp[0] = 1.0f;
  bone.matrixTemp[1] = 0.0f;
  bone.matrixTemp[2] = 0.0f;
  bone.matrixTemp[3] = 0.0f;

  bone.matrixTemp[4] = 0.0f;
  bone.matrixTemp[5] = 1.0f;
  bone.matrixTemp[6] = 0.0f;
  bone.matrixTemp[7] = 0.0f;

  bone.matrixTemp[8] = 0.0f;
  bone.matrixTemp[9] = 0.0f;
  bone.matrixTemp[10] = 1.0f;
  bone.matrixTemp[11] = 0.0f;

  bone.matrixTemp[12] = 0.0f;
  bone.matrixTemp[13] = 0.0f;
  bone.matrixTemp[14] = 0.0f;
  bone.matrixTemp[15] = 1.0f;

  if (bone.motions.empty()) {

    bone.rotation[0] = 0.0;
    bone.rotation[1] = 0.0;
    bone.rotation[2] = 0.0;
    bone.rotation[3] = 1.0;

    if (bone.parentIndex == (unsigned short)(-1)) {

      bone.matrixTemp[12] = bone.pos[0];
      bone.matrixTemp[13] = bone.pos[1];
      bone.matrixTemp[14] = bone.pos[2];

    } else {
      const Bone &parent = model->bones_[bone.parentIndex];

      bone.matrixTemp[12] = (bone.pos[0] - parent.pos[0]);
      bone.matrixTemp[13] = (bone.pos[1] - parent.pos[1]);
      bone.matrixTemp[14] = (bone.pos[2] - parent.pos[2]);
    }

  } else {

    MotionSegment ms = FindMotionSegment(frame, bone.motions);
    // Motion& motion = bone.motions[ms.m0];

    Quaternion motionRot;
    float motionPos[3];
    InterpolateMotion(motionRot, motionPos, bone.motions, ms, frame);

    bone.rotation[0] = motionRot.x;
    bone.rotation[1] = motionRot.y;
    bone.rotation[2] = motionRot.z;
    bone.rotation[3] = motionRot.w;
    Quaternion q;
    q.x = bone.rotation[0];
    q.y = bone.rotation[1];
    q.z = bone.rotation[2];
    q.w = bone.rotation[3];
    QuatToMatrix(bone.matrixTemp, q);

    if (bone.parentIndex == (unsigned short)(-1)) {

      // Quaternion q;
      // q.x = bone.rotation[0];
      // q.y = bone.rotation[1];
      // q.z = bone.rotation[2];
      // q.w = bone.rotation[3];
      // QuatToMatrix(bone.matrixTemp, q);

      bone.matrixTemp[12] = bone.pos[0] + motionPos[0];
      bone.matrixTemp[13] = bone.pos[1] + motionPos[1];
      bone.matrixTemp[14] = bone.pos[2] + motionPos[2];
      // printf("parent:rot = %f, %f, %f, %f\n", motion.rotation[0],
      // motion.rotation[1], motion.rotation[2]);
    } else {
      const Bone &parent = model->bones_[bone.parentIndex];

      //// bone.t = parent.r * (bone.t - parent.t + motion.t);
      //// bone.r = parent.r * motion.r
      // Quaternion q, q0, q1;

      // q0.x = motion.rotation[0];
      // q0.y = motion.rotation[1];
      // q0.z = motion.rotation[2];
      // q0.w = motion.rotation[3];

      // q1.x = parent.rotation[0];
      // q1.y = parent.rotation[1];
      // q1.z = parent.rotation[2];
      // q1.w = parent.rotation[3];

      // QMult(q, q0, q1);
      // printf("q0 = %f, %f, %f, %f\n", q0.x, q0.y, q0.z, q0.w);
      // printf("q1 = %f, %f, %f, %f\n", q1.x, q1.y, q1.z, q1.w);
      // printf("q  = %f, %f, %f, %f\n", q.x, q.y, q.z, q.w);

      // bone.rotation[0] = q.x;
      // bone.rotation[1] = q.y;
      // bone.rotation[2] = q.z;
      // bone.rotation[3] = q.w;

      // QuatToMatrix(bone.matrixTemp, q);

      Vector3 v;
      v.x = (bone.pos[0] - parent.pos[0]) + motionPos[0];
      v.y = (bone.pos[1] - parent.pos[1]) + motionPos[1];
      v.z = (bone.pos[2] - parent.pos[2]) + motionPos[2];

      bone.matrixTemp[12] = v.x;
      bone.matrixTemp[13] = v.y;
      bone.matrixTemp[14] = v.z;
    }
  }
};

static void Update() {
	
#if 1
  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    if (b.parentIndex != 0xFFFF) {
		//printf("Bone is Broken.\n");
      //assert(b.parentIndex < i);
    }

    SetBoneMatrix(i, b, current_frame);
    b.updated = false;
  }

  UpdateIK();

  // Clear update flag
  for (int i = 0; i < model->bones_.size(); i++) {
    model->bones_[i].updated = false;
  }

  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    UpdateBoneMatrix(b);
  }

#if 0
  // Make position relative
  for (int i = 0; i < model->bones_.size(); i++) {
    Bone& b = model->bones_[i];
    b.matrix[12] -= b.pos[0];
    b.matrix[13] -= b.pos[1];
    b.matrix[14] -= b.pos[2];
  }
#endif

  VertexTransform(renderVertices);

#else
  for (int i = 0; i < model->iks_.size(); i++) {
    IK &ik = model->iks_[i];
    Bone &p = model->bones_[ik.boneIndex];
    UpdateBone(ik.boneIndex, p, current_frame);

    if (ik.childBoneIndices.empty())
      continue;

    for (int k = 0; k < ik.childBoneIndices.size(); k++) {
      Bone &b = model->bones_[ik.childBoneIndices[k]];
      UpdateBone(ik.childBoneIndices[k], b, current_frame);
    }
  }
#endif
}

/*static void ApplyGLNormal(const Vertex &v[3])
{
	Vector d1 , d2;
	d1.x = v[1].x - v[0].x;
	d1.y = v[1].y - v[0].y;
	d1.z = v[1].z - v[0].z;
	
	d2.x = v[2].x - v[0].x;
	d2.y = v[2].y - v[0].y;
	d2.z = v[2].z - v[0].z;

	Vector cross_product;
	cross_product.x = d1.y*d2.z - d1.z*d2.y;
	cross_product.y = d1.z*d2.x - d1.x*d2.z;
	cross_product.z = d1.x*d2.y - d1.y*d2.x;

	float distance = sqrt(	(cross_product.x*cross_product.x) +
				(cross_product.y*cross_product.y) +
				(cross_product.z*cross_product.z));
	
	Vector normal;
	normal.x = cross_product.x / distance;
	normal.y = cross_product.y / distance;
	normal.z = cross_product.z / distance;

	glNormal3f(normal.x , normal.y , normal.z);
}
*/
int t=5;
bool first_frame=true;
int tid=0;
static void DrawMesh() {
  
  
	
	
  glEnable(GL_NORMALIZE);
  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,GL_REPLACE);
  
  glCullFace(GL_FRONT);
  glEnable(GL_CULL_FACE);
  
  
  
  /*
   * float diffuse[3];
  float alpha;
  float specularity;
  float specular[3];
  float ambient[3];
   * */
  int ind=0;
  for( int i = 0; i <= model->materials_.size() - 1  ;i++){
	  PMDMaterial mat=model->materials_[i];
	  int tex_id=model->texids_[i];
	  //mat.alpha
	  GLfloat diff[] = {mat.diffuse[0],mat.diffuse[1],mat.diffuse[2],1};
	  GLfloat ambi[] = {mat.ambient[0],mat.ambient[1],mat.ambient[2],1};
	  GLfloat spec[] = {mat.specular[0],mat.specular[1],mat.specular[2],1};
	  glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diff);
	  glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambi);
	  glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,spec);
	  glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,mat.specularity);
	  
	  if(0x100 & mat.edge_flag == 0x100){
		  glDisable(GL_CULL_FACE);
	  }
	  else{
		  glEnable(GL_CULL_FACE);
	  }
	  
	  if(tex_id>0){
		  glEnable(GL_TEXTURE_2D);
		  glBindTexture(GL_TEXTURE_2D , tex_id);
	  }
	  else{
		  //glDisable(GL_TEXTURE_2D);
		  //glColor3f(collist[i%7][0], collist[i%7][1], collist[i%7][2]);
	  }
	  
	  glBegin(GL_TRIANGLES);
	  for(int j=0;j<mat.vertex_count && ind < model->indices_.size();j++,ind++){
		  int idx = model->indices_[ind];
		  glTexCoord2f (model->vertices_[idx].uv[0],1-model->vertices_[idx].uv[1]);
		  glVertex3f(renderVertices[3 * idx + 0], renderVertices[3 * idx + 1],
				   -renderVertices[3 * idx + 2]);
		  //glNormal3f(); TODO
		  
	  }
	  glEnd();
	  
  }
  
  
  /*
  int mit=model->materials_.size();
  int mco=0;
  for (int i = 0; i < model->indices_.size(); i++) {
	if(mco<=0){
		mco=model->materials_[--mit].vertex_count;
	}
	
    int idx = model->indices_[i];
    // printf("v[%d] = %f, %f, %f\n",
    //  i,
    //  renderVertices[3*idx+0],
    //  renderVertices[3*idx+1],
    //  renderVertices[3*idx+2]);
    int cidx = mit%t;//model->vertices_[idx].bone[0] % 7;
    int tid=model->texids_[mit];
    if(tid>0){
		//glEnable(GL_TEXTURE_2D);
		
		glBindTexture(GL_TEXTURE_2D  , tid);
		glTexCoord2f (model->vertices_[idx].uv[0],model->vertices_[idx].uv[1]);
		//glColor3f(0,0,0);
	}
	else{
		//glDisable(GL_TEXTURE_2D);
		glColor3f(collist[cidx][0], collist[cidx][1], collist[cidx][2]);
		//glColor3f(model->materials_[mit].diffuse[0],model->materials_[mit].diffuse[1],model->materials_[mit].diffuse[2]);
	}
    // printf("cidx = %d\n", cidx);
    
    glVertex3f(renderVertices[3 * idx + 0], renderVertices[3 * idx + 1],
               -renderVertices[3 * idx + 2]);
    mco--;
    
    
    // glVertex3f(model->vertices_[idx].pos[0],
    //           model->vertices_[idx].pos[1],
    //           -model->vertices_[idx].pos[2]);
  }
  //glVertex3f(renderVertices[3 * idx + 0], renderVertices[3 * idx + 1],
  //             -renderVertices[3 * idx + 2]);
  */
  
  
  
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glDisable(GL_NORMALIZE);
  glDisable(GL_TEXTURE_2D);
}

static void DrawBoneOriginal() {
  glDisable(GL_LIGHTING);
  glLineWidth(3);
  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];

    if (b.parentIndex == 0xFFFF) {
      glBegin(GL_POINTS);
      if (b.isLeg) {
        glColor3f(1.0f, 1.0f, 1.0f);
      } else {
        glColor3f(1.0f, 0.0f, 0.0f);
      }
      glVertex3f(b.pos[0], b.pos[1], -b.pos[2]);
      glEnd();
    } else {
      Bone &p = model->bones_[b.parentIndex];

      glBegin(GL_LINES);
      glColor3f(0.0f, 0.0f, 1.0f);
      glVertex3f(p.pos[0], p.pos[1], -p.pos[2]);
      if (b.isLeg) {
        glColor3f(1.0f, 1.0f, 1.0f);
      } else {
        glColor3f(0.0f, 1.0f, 0.0f);
      }
      glVertex3f(b.pos[0], b.pos[1], -b.pos[2]);
      glEnd();
    }
  }
  glEnable(GL_LIGHTING);
}

static void DrawBone() {
  glDisable(GL_LIGHTING);
  glLineWidth(3);
  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];

    if (b.parentIndex == 0xFFFF) {
      glBegin(GL_POINTS);
      if (b.isLeg) {
        glColor3f(1.0f, 1.0f, 1.0f);
      } else {
        glColor3f(1.0f, 0.0f, 0.0f);
      }
      glVertex3f(b.matrix[12], b.matrix[13], -b.matrix[14]);
      glEnd();
    } else {
      Bone &p = model->bones_[b.parentIndex];

      glBegin(GL_LINES);
      glColor3f(0.0f, 0.0f, 1.0f);
      glVertex3f(p.matrix[12], p.matrix[13], -p.matrix[14]);
      if (b.isLeg) {
        glColor3f(1.0f, 1.0f, 1.0f);
      } else {
        glColor3f(0.0f, 1.0f, 0.0f);
      }
      glVertex3f(b.matrix[12], b.matrix[13], -b.matrix[14]);
      glEnd();
    }
  }
  glEnable(GL_LIGHTING);
}

static void DrawIK() {
  glDisable(GL_LIGHTING);
  glPointSize(5.0);
  for (int i = 0; i < model->iks_.size(); i++) {
    IK &ik = model->iks_[i];
    Bone &root = model->bones_[ik.boneIndex];

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(root.matrixTemp[12], root.matrixTemp[13], root.matrixTemp[14]);
    glEnd();

    if (ik.childBoneIndices.empty())
      continue;

    Bone *p = &root;

    for (int k = 0; k < ik.childBoneIndices.size(); k++) {
      Bone &b = model->bones_[ik.childBoneIndices[k]];
      // assert(!b.motions.empty());

      float bpos[3];
      bpos[0] = b.matrixTemp[12];
      bpos[1] = b.matrixTemp[13];
      bpos[2] = b.matrixTemp[14];
      // printf("[%d] pos = %f, %f, %f\n", ik.childBoneIndices[k], pos[0],
      // pos[1], pos[2]);

      float ppos[3];
      ppos[0] = p->matrixTemp[12];
      ppos[1] = p->matrixTemp[13];
      ppos[2] = p->matrixTemp[14];

      glBegin(GL_LINES);
      glColor3f(0.0f, 1.0f, 0.0f);
      glVertex3f(bpos[0], bpos[1], -bpos[2]);
      glColor3f(0.0f, 1.0f, 1.0f);
      glVertex3f(ppos[0], ppos[1], -ppos[2]);
      glEnd();

      p = &b;
    }
  }
  glEnable(GL_LIGHTING);
}

static void DrawAxis() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glLineWidth(2.0);

  glBegin(GL_LINES);
  // x
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);

  // y
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);

  // z
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();

  glLineWidth(1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

void build_rot_matrix(GLfloat m[4][4]) {
  /* get rotation matrix */
  build_rotmatrix(m, curr_quat);
}

void display() {
  GLfloat m[4][4];

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* get rotation matrix */
  build_rot_matrix(m);

  // set_orthoview_pass(width, height);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(view_org[0], view_org[1]+0.5, view_org[2], view_tgt[0], view_tgt[1]+0.5,
            view_tgt[2], 0, 1, 0); /* Y up */

  glMultMatrixf(&(m[0][0]));

  // draw scene bounding box
  glPushMatrix();
  glColor3f(1.0, 1.0, 1.0);
  glScalef(center[0], center[1], center[2]);
  glutWireCube(2.0 / scenesize);
  glPopMatrix();

  glScalef(1.0 / scenesize, 1.0 / scenesize, 1.0 / scenesize);

  Update();

  // overlay
  DrawAxis();

  // DrawIK();
  DrawMesh();

  //DrawBone();
  //DrawBoneOriginal();

  glutSwapBuffers();
}

void reshape(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, (float)w / (float)h, 0.1f, 50.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  width = w;
  height = h;
}

void animate() {
  sub_frame++;
  if (sub_frame >= frame_step) {
    sub_frame = 0;
    current_frame++;
  }
  //FPS : 30
static int lastUpdate = 0;
static int frames = 0;
char buf[20];

glutPostRedisplay(); // calls your display callback function
glutSwapBuffers();

int currentTime = glutGet( GLUT_ELAPSED_TIME );
frames++;

// is the time difference between lastUpdate and current time > one second ( 1000 ms )?
if ( ( currentTime - lastUpdate ) >= 1000 ){

sprintf( buf, "FPS: %d", frames );
glutSetWindowTitle( buf );
frames = 0;
lastUpdate = currentTime;

}
//End FPS : 30

  printf("\rFrame: %d", current_frame);
  fflush(stdout);
  // if (spinning) {
  //	add_quats(prev_quat, curr_quat, curr_quat);
  //}

  //glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
  int mod = glutGetModifiers();
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    trackball(prev_quat, 0, 0, 0, 0); /* initialize */
    // glutIdleFunc(NULL);
    mouse_moving = 1;
    mouse_x = x;
    mouse_y = y;
    spinning = 0;
  }

  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    spinning = 0;
    mouse_r_pressed = 0;
    mouse_m_pressed = 0;
    mouse_moving = 0;
  }

  if ((button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) ||
      (button == GLUT_LEFT_BUTTON && (mod & GLUT_ACTIVE_CTRL))) {
    mouse_r_pressed = 1;
    mouse_m_pressed = 0;
    mouse_moving = 1;
    mouse_x = x;
    mouse_y = y;
    spinning = 0;
  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
    spinning = 0;
    mouse_r_pressed = 0;
    mouse_m_pressed = 0;
    mouse_moving = 0;
  }

  if ((button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) ||
      (button == GLUT_LEFT_BUTTON && (mod & GLUT_ACTIVE_SHIFT))) {
    mouse_m_pressed = 1;
    mouse_r_pressed = 0;
    mouse_moving = 1;
    mouse_x = x;
    mouse_y = y;
    spinning = 0;
  }

  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP) {
    spinning = 0;
    mouse_m_pressed = 0;
    mouse_r_pressed = 0;
    mouse_moving = 0;
  }

  glutPostRedisplay();
}

void motion(int x, int y) {
  float w = 1;
  float tmp[4];

  if (mouse_moving) {
    if (mouse_r_pressed) {
      view_org[2] += 1.0 * ((float)y - mouse_y) / (float)height;
    } else if (mouse_m_pressed) {
      view_org[0] += 1.0 * zoom * (mouse_x - (float)x) / (float)width;
      view_org[1] += 1.0 * zoom * ((float)y - mouse_y) / (float)height;
      view_tgt[0] += 1.0 * zoom * (mouse_x - (float)x) / (float)width;
      view_tgt[1] += 1.0 * zoom * ((float)y - mouse_y) / (float)height;
    } else {
      trackball(prev_quat, w * (2.0 * mouse_x - width) / width,
                w * (height - 2.0 * mouse_y) / height,
                w * (2.0 * x - width) / width, w * (height - 2.0 * y) / height);
      add_quats(prev_quat, curr_quat, curr_quat);
    }

    mouse_x = x;
    mouse_y = y;
    spinning = 1;

    // glutIdleFunc(animate);
  }

  glutPostRedisplay();
}

void keyboard(unsigned char k, int x, int y) {
  bool redraw = false;
  bool rebuildBRDF = false;

  static int presetnum = 0;

  switch (k) {
  case 27: /* ESC */
  case 'q':
    exit(0);
    break;
  case ' ': /* space */
    /* reset view */
    trackball(curr_quat, 0.0, 0.0, 0.0, 0.0);
    mouse_moving = 0;
    spinning = 0;
    current_frame = 0;
    sub_frame = 0;
    view_org[0] = view_org[1] = 0.0;
    view_org[2] = 5.0;
    view_tgt[0] = view_tgt[1] = view_tgt[2] = 0.0;
    break;
  case '+': /* space */
    /* reset view */
    view_org[2] ++;
    break;
  case '-': /* space */
    /* reset view */
    view_org[2] --;
    break;
  case 'a': /* space */
    /* reset view */
    t ++;printf("t=%d\n",t);
    break;
  case 's': /* space */
    /* reset view */
    t --;printf("t=%d\n",t);
    break;
  default:
    break;
  }

  if (redraw) {
    display();
  }
}

void load(char *pmdmodel, char *vmdmodel) {
  PMDReader pmdreader;
  model = pmdreader.LoadFromFile(pmdmodel);
  assert(model);

  VMDReader vmdreader;
  anim = vmdreader.LoadFromFile(vmdmodel);
  assert(anim);

  MMDScene *scene = new MMDScene();
  scene->SetModel(model);
  scene->AttachAnimation(anim);

  renderVertices = new float[3 * model->vertices_.size()];

  DumpIK();
  DumpBone();
}

void init() {
  /*
   * global variable initialization
   */
  width = WINDOW_WIDTH;
  height = WINDOW_HEIGHT;
  mouse_moving = 0;
  spinning = 0;
  zoom = 1;

  // current_frame = 0;
  sub_frame = 0;

  view_org[0] = view_org[1] = 0.0;
  view_org[2] = 3.0;
  view_tgt[0] = view_tgt[1] = view_tgt[2] = 0.0;

  trackball(curr_quat, 0.0, 0.0, 0.0, 0.0);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  
  if(first_frame){
	  
	  first_frame=false;
	  model->texids_.resize(model->materials_.size());
	  int sum_indices=0;
	  for (int i = 0; i < model->materials_.size(); i++) {		  
		  char tex_file[256]="";
		  char spa_file[256]=""; 
		  
		  char * str =model->materials_[i].texture_filename;
		  char * pch;
		  pch = strstr(str,"*");
		  if(pch==NULL){
			  strncpy (tex_file,str,strlen(str));
			  printf("tex_file : %s\n",tex_file);
		  }
		  else{
			  strncpy (tex_file,str,pch-str);
			  strncpy (spa_file,pch+1,strlen(str)-(pch+1-str));
			  printf("tex_file : %s \t spa_fie : %s\n",tex_file,spa_file);
		  }
		  char tmp_path[256];//"hzeo_kaitoV3formal/";
		  strcpy(tmp_path,tex_path);
		  //printf("%s\n",strcat(tmp_path,tex_file));
		  {
			  strcat(tmp_path,tex_file);
			model->texids_[i]= SOIL_load_OGL_texture
								(
									tmp_path,
									SOIL_LOAD_AUTO,
									SOIL_CREATE_NEW_ID,
									SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
								);
								
							/* check for an error during the load process */
							if( 0 == model->texids_[i] )
							{
								printf( "SOIL loading error: '%s'\n", SOIL_last_result() );
							}

			
		  }
		  sum_indices+=model->materials_[i].vertex_count;
		  printf("> %d : %d\n",i,model->texids_[i]);
		  //if(model->texids_[i]>0)
			//tid=model->texids_[i];
	  }
	  
	  printf("#%d %d\n\n",model->indices_.size(),sum_indices);
  }
  
  
}

int main(int argc, char **argv) {
  if (argc < 3) {
    printf("Usage: %s input.pmd input.vmd\n", argv[0]);
    exit(-1);
  }
  if(argc==4){
	  strncpy( tex_path,argv[3],strlen(argv[3]));
  }

  load(argv[1], argv[2]);

  glutInit(&argc, argv);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutCreateWindow(argv[0]);

  init();

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutIdleFunc(animate);

  glutMainLoop();

  return 0;
}
