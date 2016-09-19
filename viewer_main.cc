#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <float.h>

#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <algorithm>
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

// image loader
//#include "stb_image.c"

#ifdef ENABLE_GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/euler_angles.hpp>

#if GLM_VERSION >= 96
    // glm::rotate changed from degrees to radians in GLM 0.9.6
    // https://glm.g-truc.net/0.9.6/updates.html
    #define GLM_ROTATE(m, a, v) glm::rotate((m), glm::radians(a), (v))
#else
    #define GLM_ROTATE(m, a, v) glm::rotate((m), (a), (v))
#endif

#endif

using namespace mmd;

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define LARGE_NUMBER 1000000

#if defined(ENABLE_GLM)
#define ORIENT_ROLL(v)  v[0]
#define ORIENT_PITCH(v) v[1]
#define ORIENT_YAW(v)   v[2]

#define VEC_LEFT    glm::vec3(1, 0, 0)
#define VEC_UP      glm::vec3(0, 1, 0)
#define VEC_FORWARD glm::vec3(0, 0, 1)

#ifdef ENABLE_EULER_CAMERA
#define MAX_PITCH  89.999
#define MIN_PITCH -89.999
#endif

#endif

#define MAX_BUF_LEN 20

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
static GLfloat view_fov = 45.0f;
static float bmin[3] = {-1, -1, -1}, bmax[3] = {1, 1, 1};
static float scenesize = 20.0f;
static int current_frame = 0;
static int sub_frame = 0;
static int frame_step = 1; // less = faster playback.
static float eye_distance = 2.0f;

#if defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA)
bool left_mouse_down = false, right_mouse_down = false;
glm::vec2 prev_mouse_coord, mouse_drag;
glm::vec3 prev_orient, orient, orbit_speed = glm::vec3(0, -0.5, -0.5);
float prev_orbit_radius = 0, orbit_radius = 3, dolly_speed = 0.1;
#endif

static bool do_animate = true;
static bool draw_axis = false;
static bool draw_ik = false;
static bool draw_mesh = true;
static int draw_bbox_mode = 0;
static bool draw_bones = true;
static bool draw_wireframe = false;
static bool print_bone_info = false;
static bool draw_bullet_scene = false;
static bool draw_bullet_result = false;
static int split_screen_vr_mode = 0;

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

#if defined(ENABLE_GLM)
#define EPSILON 0.0001

#define SIGN(x) (!(x) ? 0 : (((x) > 0) ? 1 : -1))

static glm::vec3 OrientToOffset(glm::vec3 orient) {
  glm::mat4 pitch = GLM_ROTATE(glm::mat4(1), ORIENT_PITCH(orient), VEC_LEFT);
  glm::mat4 yaw = GLM_ROTATE(glm::mat4(1), ORIENT_YAW(orient), VEC_UP);
  return glm::vec3(yaw*pitch*glm::vec4(VEC_FORWARD, 1));
}

glm::vec3 OffsetToOrient(glm::vec3 offset) {
  offset = glm::normalize(offset);
  glm::vec3 t(offset.x, 0, offset.z); // flattened offset
  t = glm::normalize(t);
  glm::vec3 r(0, glm::angle(t, offset), glm::angle(t, VEC_FORWARD));
  if(static_cast<float>(fabs(offset.x)) < EPSILON && static_cast<float>(fabs(offset.z)) < EPSILON) {
    ORIENT_PITCH(r) = -SIGN(offset.y)*glm::radians(90.0f);
    ORIENT_YAW(r) = 0; // undefined
    return r;
  }
  if(offset.x < 0) ORIENT_YAW(r)   *= -1;
  if(offset.y > 0) ORIENT_PITCH(r) *= -1;
  return r;
}

#ifdef ENABLE_EULER_CAMERA
static void UpdateCameraParams() {
  glm::vec3 view_target(scene->static_center.x, scene->static_center.y, scene->static_center.z);
  glm::vec3 view_origin = view_target+OrientToOffset(orient)*orbit_radius;
  view_org[0] = view_origin.x;
  view_org[1] = view_origin.y;
  view_org[2] = view_origin.z;
  view_tgt[0] = scene->static_center.x;
  view_tgt[1] = scene->static_center.y;
  view_tgt[2] = scene->static_center.z;
}
#endif

#endif

static void PrintBitmapString(void* font, const char* s) {
  if (s && strlen(s)) {
    while (*s) {
      glutBitmapCharacter(font, *s);
      s++;
    }
  }
}

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
#define PI 3.1415926535897932

float fallHeight = 50;
int simStep = 0;
int maxSimSteps = 500;

// http://bulletphysics.org/Bullet/BulletFull/classbtIDebugDraw.html
class GLDebugDrawer : public btIDebugDraw
{
public:
  GLDebugDrawer() : debugMode_() {}
  virtual ~GLDebugDrawer() {}
  virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
  virtual void drawContactPoint(const btVector3&, const btVector3&, btScalar, int, const btVector3&) {}
  virtual void reportErrorWarning(const char*) {}
  virtual void draw3dText(const btVector3&, const char*) {}
  virtual void setDebugMode(int debugMode) { debugMode_ = debugMode; }
  virtual int getDebugMode() const { return debugMode_; }
  static GLDebugDrawer* instance() {
    static GLDebugDrawer instance_;
    return &instance_;
  }
private:
  int debugMode_;
};

// http://stackoverflow.com/questions/14008295/how-to-implement-the-btidebugdraw-interface-of-bullet-in-opengl-4-0
void GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
  glBegin(GL_LINES);
    glColor3f(color.getX(), color.getY(), color.getZ());
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glColor3f(color.getX(), color.getY(), color.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
  glEnd();
}

btBroadphaseInterface*               broadphase;
btDefaultCollisionConfiguration*     collisionConfiguration;
btCollisionDispatcher*               dispatcher;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld*             dynamicsWorld;

struct BulletDynamicObject_t {
  btCollisionShape*     shape;
  btDefaultMotionState* motionState;
  btRigidBody*          rigidBody;
  Bone*                 followBone;
};

BulletDynamicObject_t ground, fall;
std::vector<BulletDynamicObject_t*> bullet_dynamic_objects;
std::set<std::string> bullet_follow_bone_names;

// http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
static void InitSimulation() {
  broadphase             = new btDbvtBroadphase();
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher             = new btCollisionDispatcher(collisionConfiguration);
  solver                 = new btSequentialImpulseConstraintSolver;
  dynamicsWorld          = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  ground.shape       = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  ground.motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
  ground.rigidBody   = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0, ground.motionState, ground.shape, btVector3(0, 0, 0)));
  dynamicsWorld->addRigidBody(ground.rigidBody);

  fall.shape = new btCapsuleShape(2, 4); //new btSphereShape(0.5);
  fall.motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, fallHeight, 0)));
  float fallMass = 1;
  btVector3 fallInertia(0, 0, 0);
  fall.shape->calculateLocalInertia(fallMass, fallInertia);
  fall.rigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(fallMass, fall.motionState, fall.shape, fallInertia));
  dynamicsWorld->addRigidBody(fall.rigidBody);

  // main bones
  bullet_follow_bone_names.insert("head");
  bullet_follow_bone_names.insert("neck");
  bullet_follow_bone_names.insert("upper_body");
  bullet_follow_bone_names.insert("lower_body");

  // arm bones
  bullet_follow_bone_names.insert("shoulder_L");
  bullet_follow_bone_names.insert("shoulder_R");
  bullet_follow_bone_names.insert("arm_L");
  bullet_follow_bone_names.insert("arm_R");
  bullet_follow_bone_names.insert("elbow_L");
  bullet_follow_bone_names.insert("elbow_R");
  bullet_follow_bone_names.insert("wrist_L");
  bullet_follow_bone_names.insert("wrist_R");

  // leg bones
  bullet_follow_bone_names.insert("leg_L");
  bullet_follow_bone_names.insert("leg_R");
  bullet_follow_bone_names.insert("knee_L");
  bullet_follow_bone_names.insert("knee_R");
  bullet_follow_bone_names.insert("ankle_L");
  bullet_follow_bone_names.insert("ankle_R");

  // minor bones
  bullet_follow_bone_names.insert("thumb1_L");
  bullet_follow_bone_names.insert("thumb1_R");
  bullet_follow_bone_names.insert("thumb2_L");
  bullet_follow_bone_names.insert("thumb2_R");
  bullet_follow_bone_names.insert("fore1_L");
  bullet_follow_bone_names.insert("fore1_R");
  bullet_follow_bone_names.insert("fore2_L");
  bullet_follow_bone_names.insert("fore2_R");
  bullet_follow_bone_names.insert("fore3_L");
  bullet_follow_bone_names.insert("fore3_R");
  bullet_follow_bone_names.insert("middle1_L");
  bullet_follow_bone_names.insert("middle1_R");
  bullet_follow_bone_names.insert("middle2_L");
  bullet_follow_bone_names.insert("middle2_R");
  bullet_follow_bone_names.insert("middle3_L");
  bullet_follow_bone_names.insert("middle3_R");
  bullet_follow_bone_names.insert("third1_L");
  bullet_follow_bone_names.insert("third1_R");
  bullet_follow_bone_names.insert("third2_L");
  bullet_follow_bone_names.insert("third2_R");
  bullet_follow_bone_names.insert("third3_L");
  bullet_follow_bone_names.insert("third3_R");
  bullet_follow_bone_names.insert("little1_L");
  bullet_follow_bone_names.insert("little1_R");
  bullet_follow_bone_names.insert("little2_L");
  bullet_follow_bone_names.insert("little2_R");
  bullet_follow_bone_names.insert("little3_L");
  bullet_follow_bone_names.insert("little3_R");

  // clear dynamic object lini
  for(int i = 0; i < model->bones_.size(); i++) {
    Bone* followBone = &model->bones_[i];
    followBone->bulletDynamicObject = NULL;
  }

  // assign a dynamic object to each bone
  for(int j = 0; j < model->bones_.size(); j++) {
    // identify bone endpoints
    Bone* followBone = &model->bones_[j];
    if(bullet_follow_bone_names.find(followBone->ascii_name) == bullet_follow_bone_names.end() &&
        !followBone->isChain)
    {
      // skip if not in "bullet_follow_bone_names" and not chain bone
      continue;
    }
    // chain bones link up, non-chain bones link down
    Bone* followBoneOther = &model->bones_[followBone->isChain ? followBone->parentIndex : followBone->tailIndex];

    // calculate bone dimensions
#ifdef ENABLE_GLM
    glm::vec3 bone_start(followBone->pos[0], followBone->pos[1], followBone->pos[2]);
    glm::vec3 bone_end(followBoneOther->pos[0], followBoneOther->pos[1], followBoneOther->pos[2]);
    float bone_length = glm::distance(bone_start, bone_end);
#else
    Vector3 bone_start;
    bone_start.x = followBone->pos[0];
    bone_start.y = followBone->pos[1];
    bone_start.z = followBone->pos[2];
    Vector3 bone_end;
    bone_end.x = followBoneOther->pos[0];
    bone_end.y = followBoneOther->pos[1];
    bone_end.z = followBoneOther->pos[2];
    Vector3 bone_delta;
    VSub(bone_delta, bone_end, bone_start);
    float bone_length = VLength(bone_delta);
#endif

    // configure dynamic object with mass, scripted object without mass
    BulletDynamicObject_t* bullet_dynamic_object = new BulletDynamicObject_t();
    float capsule_radius = 0;
    float capsule_length = 0;
    if(followBone->hasVertices) {
      capsule_radius = std::min(followBone->dim.x, std::min(followBone->dim.y, followBone->dim.z))*0.5;
      capsule_length = std::max(bone_length - capsule_radius*2, 0.0f);
    }
    bullet_dynamic_object->shape = new btCapsuleShape(capsule_radius, capsule_length);
    if(followBone->isChain && !followBone->isPinnedChain) {
      bullet_dynamic_object->motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
      float fallMass = 1;
      btVector3 fallInertia(0, 0, 0);
      bullet_dynamic_object->shape->calculateLocalInertia(fallMass, fallInertia);
      bullet_dynamic_object->rigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(fallMass, bullet_dynamic_object->motionState, bullet_dynamic_object->shape, fallInertia));
    } else {
      bullet_dynamic_object->motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
      bullet_dynamic_object->rigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0, bullet_dynamic_object->motionState, bullet_dynamic_object->shape, btVector3(0, 0, 0)));
    }

    // register dynamic object to world
    dynamicsWorld->addRigidBody(bullet_dynamic_object->rigidBody);
    bullet_dynamic_object->followBone = followBone;
    bullet_dynamic_objects.push_back(bullet_dynamic_object);

    followBone->bulletDynamicObject = bullet_dynamic_object;
  }

  // generate "simple chain" constraints
  // http://bulletphysics.org/mediawiki-1.5.8/index.php/Simple_Chain
  for(int k = 0; k < model->bones_.size(); k++) {
    Bone* followBone = &model->bones_[k];
    if(!followBone->isChain || followBone->isPinnedChain) {
      continue;
    }
    Bone* followBoneParent = &model->bones_[followBone->parentIndex];
    if(!followBone->bulletDynamicObject || !followBoneParent->bulletDynamicObject) {
      // DBG
      //std::cout << "SKIPPING: " << followBone->ascii_name << std::endl;
      continue;
    }

    btRigidBody* b1 = reinterpret_cast<BulletDynamicObject_t*>(followBone->bulletDynamicObject)->rigidBody;
    btRigidBody* b2 = reinterpret_cast<BulletDynamicObject_t*>(followBoneParent->bulletDynamicObject)->rigidBody;

    // calculate bone dimensions
#ifdef ENABLE_GLM
    glm::vec3 bone_start(followBone->pos[0], followBone->pos[1], followBone->pos[2]);
    glm::vec3 bone_end(followBoneParent->pos[0], followBoneParent->pos[1], followBoneParent->pos[2]);
    float bone_length = glm::distance(bone_start, bone_end);
#else
    Vector3 bone_start;
    bone_start.x = followBone->pos[0];
    bone_start.y = followBone->pos[1];
    bone_start.z = followBone->pos[2];
    Vector3 bone_end;
    bone_end.x = followBoneParent->pos[0];
    bone_end.y = followBoneParent->pos[1];
    bone_end.z = followBoneParent->pos[2];
    Vector3 bone_delta;
    VSub(bone_delta, bone_end, bone_start);
    float bone_length = VLength(bone_delta);
#endif
    // DBG
    //std::cout << "CHAIN: " << followBone->ascii_name << " ==> " << followBoneParent->ascii_name <<
    //    (followBoneParent->isPinnedChain ? "*" : "") << " (" << bone_length << ")" << std::endl;

    float pivot_offset = bone_length*0.5;

#ifdef ENABLE_DUAL_CONSTRAINT_CHAIN
    // straight from the bullet example, but with no basis in reality
    btPoint2PointConstraint* leftSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5, pivot_offset, 0), btVector3(-0.5, -pivot_offset,0));
    dynamicsWorld->addConstraint(leftSpring);

    btPoint2PointConstraint* rightSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0.5, pivot_offset, 0), btVector3(0.5, -pivot_offset, 0));
    dynamicsWorld->addConstraint(rightSpring);
#else
    // simpler setup with negligible loss of quality
    btPoint2PointConstraint* rightSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(0, pivot_offset, 0), btVector3(0, -pivot_offset, 0));
    dynamicsWorld->addConstraint(rightSpring);
#endif
  }

  // http://stackoverflow.com/questions/11985204/how-to-draw-render-a-bullet-physics-collision-body-shape
  dynamicsWorld->setDebugDrawer(GLDebugDrawer::instance());
  GLDebugDrawer::instance()->setDebugMode(true);
}

static void DeInitSimulation() {
  // free rigid bodies
  for(std::vector<BulletDynamicObject_t*>::iterator p = bullet_dynamic_objects.begin();
      p != bullet_dynamic_objects.end(); p++)
  {
    BulletDynamicObject_t* bullet_dynamic_object = *p;
    dynamicsWorld->removeRigidBody(bullet_dynamic_object->rigidBody);
    delete bullet_dynamic_object->rigidBody->getMotionState();
    delete bullet_dynamic_object->rigidBody;
    delete bullet_dynamic_object->shape;
  }
  bullet_dynamic_objects.clear();

  dynamicsWorld->removeRigidBody(fall.rigidBody);
  delete fall.rigidBody->getMotionState();
  delete fall.rigidBody;
  delete fall.shape;

  dynamicsWorld->removeRigidBody(ground.rigidBody);
  delete ground.rigidBody->getMotionState();
  delete ground.rigidBody;
  delete ground.shape;

  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
}

static void StepSimulation() {
  dynamicsWorld->stepSimulation(1 / 60.f, 10);

  // make dynamic object follow its assigned bone
  for(std::vector<BulletDynamicObject_t*>::iterator p = bullet_dynamic_objects.begin();
      p != bullet_dynamic_objects.end(); p++)
  {
    BulletDynamicObject_t* bullet_dynamic_object = *p;
    Bone* followBone = bullet_dynamic_object->followBone;
    if(!followBone) {
      continue;
    }
    if(followBone->isChain && !followBone->isPinnedChain) {
      continue;
    }

    // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
    btTransform startTransform;

#ifdef ENABLE_GLM
    // calculate bone endpoints in world space
    // chain bones link up, non-chain bones link down
    Bone* followBoneOther = &model->bones_[followBone->isChain ? followBone->parentIndex : followBone->tailIndex];
    glm::vec4 bone_start = glm::make_mat4(followBone->matrix)*glm::vec4(0, 0, 0, 1);
    glm::vec4 bone_end = glm::make_mat4(followBoneOther->matrix)*glm::vec4(0, 0, 0, 1);

    // opengl is right-handed coordinate system
    // pmd is left-handed coordinate system
    bone_start = glm::scale(glm::mat4(1), glm::vec3(1, 1, -1))*bone_start;
    bone_end = glm::scale(glm::mat4(1), glm::vec3(1, 1, -1))*bone_end;

    // point dynamic object along bone
    glm::vec3 bone_orient = OffsetToOrient(glm::vec3(bone_end - bone_start));
    glm::mat4 bone_orient_xform_glm = glm::yawPitchRoll(ORIENT_YAW(bone_orient),
                                                        ORIENT_PITCH(bone_orient),
                                                        ORIENT_ROLL(bone_orient));

    // point capsule "shape" in default euler orientation
    bone_orient_xform_glm = bone_orient_xform_glm*GLM_ROTATE(glm::mat4(1), 90.0f, VEC_LEFT);

    // convert glm matrix into bullet matrix
    btMatrix3x3 bone_orient_xform_bt;
    for(int i = 0; i<4; i++) {
      for(int j = 0; j<4; j++) {
        // glm matrix is column major, bullet matrix is row major
        bone_orient_xform_bt[i][j] = bone_orient_xform_glm[j][i];
      }
    }

    // move dynamic object to bone center
    glm::vec4 bone_center = (bone_start + bone_end)*0.5f;

    // configure dynamic object position/orientation
    startTransform = btTransform(bone_orient_xform_bt, btVector3(bone_center.x, bone_center.y, bone_center.z));
#else
    startTransform.setFromOpenGLMatrix(followBone->matrix);

    // opengl is right-handed coordinate system
    // pmd is left-handed coordinate system
    btTransform flip_z_xform(btMatrix3x3(1, 0, 0,
                                         0, 1, 0,
                                         0, 0, -1));
    startTransform = flip_z_xform*startTransform;
#endif

    // this is not enough to move a rigid body in bullet..
    bullet_dynamic_object->rigidBody->getMotionState()->setWorldTransform(startTransform);

    // required step to move a rigid body in bullet
    // re-adding the rigid body to the world also works
    bullet_dynamic_object->rigidBody->setMotionState(bullet_dynamic_object->rigidBody->getMotionState());

    // shortcut to moving "relative" position (undesired!)
    // fall.rigidBody->translate(btVector3(0, fallHeight, 0));

    // required step to wake up deactivated rigid bodies in bullet
    // dynamic rigid bodies deactivate after remaining static for a while
    bullet_dynamic_object->rigidBody->activate();
  }

  // reset falling capsule
  if(simStep > maxSimSteps || !fall.rigidBody->isActive()) {

    // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
    btTransform startTransform;

    // randomize start position
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0, fallHeight, 0));
    float axis_x = -1+2*rand()/RAND_MAX;
    float axis_y = -1+2*rand()/RAND_MAX;
    float axis_z = -1+2*rand()/RAND_MAX;
    float angle  = 2*PI*rand()/RAND_MAX;
    startTransform.setRotation(btQuaternion(axis_x, axis_y, axis_z, angle));

    // this is not enough to move a rigid body in bullet..
    fall.rigidBody->getMotionState()->setWorldTransform(startTransform);

    // required step to move a rigid body in bullet
    // re-adding the rigid body to the world also works
    fall.rigidBody->setMotionState(fall.rigidBody->getMotionState());

    // shortcut to moving "relative" position (undesired!)
    // fall.rigidBody->translate(btVector3(0, fallHeight, 0));

    // required step to wake up deactivated rigid bodies in bullet
    // dynamic rigid bodies deactivate after remaining static for a while
    fall.rigidBody->activate();

    simStep = 0;
    return;
  }
  simStep++;
}
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

static void CalculateBboxMinMax() {
  // calculate bone bbox
  for (int i = 0; i < model->bones_.size(); i++) {
    model->bones_[i].min.x =  LARGE_NUMBER;
    model->bones_[i].min.y =  LARGE_NUMBER;
    model->bones_[i].min.z =  LARGE_NUMBER;
    model->bones_[i].max.x = -LARGE_NUMBER;
    model->bones_[i].max.y = -LARGE_NUMBER;
    model->bones_[i].max.z = -LARGE_NUMBER;
    model->bones_[i].hasVertices = false;
  }
  for (int j = 0; j < model->vertices_.size(); j++) {
    PMDVertex &pv = model->vertices_[j];
    Vector3 p0;
    p0.x = pv.pos[0];
    p0.y = pv.pos[1];
    p0.z = pv.pos[2];
    unsigned short b0 = pv.bone[0];
    model->bones_[b0].min.x = std::min(model->bones_[b0].min.x, p0.x);
    model->bones_[b0].min.y = std::min(model->bones_[b0].min.y, p0.y);
    model->bones_[b0].min.z = std::min(model->bones_[b0].min.z, p0.z);
    model->bones_[b0].max.x = std::max(model->bones_[b0].max.x, p0.x);
    model->bones_[b0].max.y = std::max(model->bones_[b0].max.y, p0.y);
    model->bones_[b0].max.z = std::max(model->bones_[b0].max.z, p0.z);
    model->bones_[b0].hasVertices = true;
  }

  // calculate static scene bbox
  scene->static_min.x =  LARGE_NUMBER;
  scene->static_min.y =  LARGE_NUMBER;
  scene->static_min.z =  LARGE_NUMBER;
  scene->static_max.x = -LARGE_NUMBER;
  scene->static_max.y = -LARGE_NUMBER;
  scene->static_max.z = -LARGE_NUMBER;
  for (int k = 0; k < model->bones_.size(); k++) {
    VSub(model->bones_[k].dim, model->bones_[k].max, model->bones_[k].min);

    scene->static_min.x = std::min(scene->static_min.x, model->bones_[k].min.x);
    scene->static_min.y = std::min(scene->static_min.y, model->bones_[k].min.y);
    scene->static_min.z = std::min(scene->static_min.z, model->bones_[k].min.z);
    scene->static_max.x = std::max(scene->static_max.x, model->bones_[k].max.x);
    scene->static_max.y = std::max(scene->static_max.y, model->bones_[k].max.y);
    scene->static_max.z = std::max(scene->static_max.z, model->bones_[k].max.z);

    Vector3 axis;
    axis.x = model->bones_[k].pos[0];
    axis.y = model->bones_[k].pos[1];
    axis.z = model->bones_[k].pos[2];

    // Bone matrix is defined in absolute coordinates.
    // Pass vertex static_min/static_max in relative coordinate to bone matrix.
    VSub(model->bones_[k].max, model->bones_[k].max, axis);
    VSub(model->bones_[k].min, model->bones_[k].min, axis);
  }
  VSub(scene->static_dim, scene->static_max, scene->static_min);
  scene->static_center.x = (scene->static_min.x + scene->static_max.x) * 0.5;
  scene->static_center.y = (scene->static_min.y + scene->static_max.y) * 0.5;
  scene->static_center.z = (scene->static_min.z + scene->static_max.z) * 0.5;
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

    // Bone matrix is defined in absolute coordinates.
    // Pass a vertex in relative coordinate to bone matrix.
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

  // calculate dynamic scene bbox
  scene->dynamic_min.x =  LARGE_NUMBER;
  scene->dynamic_min.y =  LARGE_NUMBER;
  scene->dynamic_min.z =  LARGE_NUMBER;
  scene->dynamic_max.x = -LARGE_NUMBER;
  scene->dynamic_max.y = -LARGE_NUMBER;
  scene->dynamic_max.z = -LARGE_NUMBER;
  for (int j = 0; j < model->vertices_.size(); j++) {
    scene->dynamic_min.x = std::min(scene->dynamic_min.x, vbuffer[3 * j + 0]);
    scene->dynamic_min.y = std::min(scene->dynamic_min.y, vbuffer[3 * j + 1]);
    scene->dynamic_min.z = std::min(scene->dynamic_min.z, vbuffer[3 * j + 2]);
    scene->dynamic_max.x = std::max(scene->dynamic_max.x, vbuffer[3 * j + 0]);
    scene->dynamic_max.y = std::max(scene->dynamic_max.y, vbuffer[3 * j + 1]);
    scene->dynamic_max.z = std::max(scene->dynamic_max.z, vbuffer[3 * j + 2]);
  }
  VSub(scene->dynamic_dim, scene->dynamic_max, scene->dynamic_min);
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
      assert(b.parentIndex < i);
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

#ifdef ENABLE_BULLET
  if(do_animate) {
    // move simulation here so there's up-to-date rigid body coordinates right before VertexTransform
    StepSimulation();
  }

  // write to bone matrix elements corresponding with translation
  for(std::vector<BulletDynamicObject_t*>::iterator p = bullet_dynamic_objects.begin();
      p != bullet_dynamic_objects.end(); p++)
  {
    BulletDynamicObject_t* bullet_dynamic_object = *p;
    Bone* followBone = bullet_dynamic_object->followBone;
    if(!followBone) {
      continue;
    }
    if(followBone->isChain && !followBone->isPinnedChain) {
      btTransform worldTransform;
      bullet_dynamic_object->rigidBody->getMotionState()->getWorldTransform(worldTransform);
      btVector3 worldPos = worldTransform.getOrigin();

      // Bone matrix is defined in absolute coordinates.
      followBone->matrix[12] = worldPos.getX();
      followBone->matrix[13] = worldPos.getY();
      followBone->matrix[14] = -worldPos.getZ();
      continue;
    }
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

static void DrawMesh() {
  glDisable(GL_LIGHTING);
  //glDisable(GL_DEPTH_TEST);

  glBegin(GL_TRIANGLES);
  int face_count = model->indices_.size() / 3;
  for (int i = 0; i < face_count; i++) {
    int idx0 = model->indices_[3 * i + 0];
    int idx1 = model->indices_[3 * i + 2];
    int idx2 = model->indices_[3 * i + 1];
    // printf("v[%d] = %f, %f, %f\n",
    //  i,
    //  renderVertices[3*idx+0],
    //  renderVertices[3*idx+1],
    //  renderVertices[3*idx+2]);
    int cidx = model->vertices_[idx0].bone[0] % 7;
    // printf("cidx = %d\n", cidx);
    glColor3f(collist[cidx][0], collist[cidx][1], collist[cidx][2]);
    glVertex3f(renderVertices[3 * idx0 + 0], renderVertices[3 * idx0 + 1],
               -renderVertices[3 * idx0 + 2]);
    glVertex3f(renderVertices[3 * idx1 + 0], renderVertices[3 * idx1 + 1],
               -renderVertices[3 * idx1 + 2]);
    glVertex3f(renderVertices[3 * idx2 + 0], renderVertices[3 * idx2 + 1],
               -renderVertices[3 * idx2 + 2]);
    // glVertex3f(model->vertices_[idx].pos[0],
    //           model->vertices_[idx].pos[1],
    //           -model->vertices_[idx].pos[2]);
  }
  glEnd();

  //glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawBoneOriginal() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glLineWidth(3.0);

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

  glLineWidth(1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawBone() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glLineWidth(3.0);

  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];

    // read from bone matrix elements corresponding with translation
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

  glLineWidth(1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawIK() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glPointSize(5.0);

  for (int i = 0; i < model->iks_.size(); i++) {
    IK &ik = model->iks_[i];
    Bone &root = model->bones_[ik.boneIndex];

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(root.matrixTemp[12], root.matrixTemp[13], -root.matrixTemp[14]);
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

  glPointSize(1.0);
  glEnable(GL_DEPTH_TEST);
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

static void DrawBoneAxis() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    glPushMatrix();
    glScalef(1, 1, -1);
    glMultMatrixf(b.matrix);
    DrawAxis();
    glPopMatrix();
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawBoneBbox() {
  glDisable(GL_LIGHTING);
  //glDisable(GL_DEPTH_TEST);

  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    glPushMatrix();
    glScalef(1, 1, -1);
    glMultMatrixf(b.matrix);
    glTranslatef(b.min.x, b.min.y, b.min.z);
    glScalef(b.dim.x, b.dim.y, b.dim.z);
    glTranslatef(0.5, 0.5, 0.5);
    int cidx = i % 7;
    glColor3f(collist[cidx][0], collist[cidx][1], collist[cidx][2]);
    glutWireCube(1);
    glPopMatrix();
  }

  //glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawSceneBbox() {
  glDisable(GL_LIGHTING);
  //glDisable(GL_DEPTH_TEST);

  glPushMatrix();
  glScalef(1, 1, -1);
  glTranslatef(scene->dynamic_min.x, scene->dynamic_min.y, scene->dynamic_min.z);
  glScalef(scene->dynamic_dim.x, scene->dynamic_dim.y, scene->dynamic_dim.z);
  glTranslatef(0.5, 0.5, 0.5);
  glColor3f(1, 1, 1);
  glutWireCube(1);
  glPopMatrix();

  //glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void PrintBoneInfo() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  for (int i = 0; i < model->bones_.size(); i++) {
    Bone &b = model->bones_[i];
    if(b.ascii_name.empty()) {
      continue;
    }
    glPushMatrix();
    glScalef(1, 1, -1);
    glMultMatrixf(b.matrix);
    int cidx = i % 7;
    glColor3f(collist[cidx][0], collist[cidx][1], collist[cidx][2]);
    glRasterPos2f(0, 0);
    static char buf[MAX_BUF_LEN];
    sprintf(buf, "%s", b.ascii_name.c_str());
    PrintBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, buf);
    glPopMatrix();
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

#ifdef ENABLE_BULLET
static void DrawBulletScene() {
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // http://stackoverflow.com/questions/11985204/how-to-draw-render-a-bullet-physics-collision-body-shape
  dynamicsWorld->debugDrawWorld();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

static void DrawBulletResult() {
  glDisable(GL_LIGHTING);
  //glDisable(GL_DEPTH_TEST);

  for(std::vector<BulletDynamicObject_t*>::iterator p = bullet_dynamic_objects.begin();
      p != bullet_dynamic_objects.end(); p++)
  {
    BulletDynamicObject_t* bullet_dynamic_object = *p;
    Bone* followBone = bullet_dynamic_object->followBone;
    if(!followBone) {
      continue;
    }
    if(followBone->isChain) {
      btTransform worldTransform;
      bullet_dynamic_object->rigidBody->getMotionState()->getWorldTransform(worldTransform);
      glPushMatrix();
      btVector3 worldPos = worldTransform.getOrigin();
      glTranslatef(worldPos.getX(), worldPos.getY(), worldPos.getZ());
      glColor3f(1, 0, 0);
      glutWireCube(1);
      glPopMatrix();
    }
  }

  //glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

#endif

void build_rot_matrix(GLfloat m[4][4]) {
  /* get rotation matrix */
  build_rotmatrix(m, curr_quat);
}

void display_for_one_eye(int eye_index, float eye_distance) {
  float w = width;
  float h = height;

  float parallax_offset = 0;

  switch(eye_index) {
  case 0: // cycloptic (no VR)
    glViewport(0, 0, w, h);
    break;
  case 1: // left eye
    w = width*0.5;
    h = height;
    glViewport(0, 0, w, h);
    parallax_offset = eye_distance*0.5;
    break;
  case 2: // right eye
    w = width*0.5;
    h = height;
    glViewport(w, 0, w, h);
    parallax_offset = -eye_distance*0.5;
    break;
  }

  glMatrixMode(GL_PROJECTION);

#ifdef ENABLE_GLM
  glm::mat4 projection = glm::perspective(view_fov, (float)w / (float)h, 0.1f, 50.0f);
  glLoadMatrixf(glm::value_ptr(projection));
#else
  glLoadIdentity();
  gluPerspective(view_fov, (float)w / (float)h, 0.1f, 50.0f);
#endif

#if !(defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA))
  parallax_offset /= scenesize;
#endif

  if(eye_index) {
    glTranslatef(parallax_offset, 0, 0);
  }

  glMatrixMode(GL_MODELVIEW);

#ifdef ENABLE_GLM
  glm::vec3 origin(view_org[0], view_org[1], view_org[2]);
  glm::vec3 target(view_tgt[0], view_tgt[1], view_tgt[2]);
  glm::mat4 model_view = glm::lookAt(origin, target, VEC_UP);
  glLoadMatrixf(glm::value_ptr(model_view));
#else
  glLoadIdentity();
  gluLookAt(view_org[0], view_org[1], view_org[2], view_tgt[0], view_tgt[1],
            view_tgt[2], 0, 1, 0); /* Y up */
#endif

#if !(defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA))
  GLfloat m[4][4];

  /* get rotation matrix */
  build_rot_matrix(m);

  glMultMatrixf(&(m[0][0]));
  glScalef(1.0 / scenesize, 1.0 / scenesize, 1.0 / scenesize);
#endif

  DrawAxis();

  // overlay
  if(draw_axis) {
    DrawBoneAxis();
  }

  if(draw_ik) {
    DrawIK();
  }

  if(draw_mesh) {
    DrawMesh();
  }

  switch(draw_bbox_mode) {
  case 0:
    break;
  case 1:
    DrawSceneBbox();
    break;
  case 2:
    DrawSceneBbox();
    DrawBoneBbox();
    break;
  case 3:
    DrawBoneBbox();
    break;
  }

  if(draw_bones) {
    DrawBone();
    //DrawBoneOriginal();
  }

  if(print_bone_info) {
    PrintBoneInfo();
  }

#ifdef ENABLE_BULLET
  if(draw_bullet_scene) {
    DrawBulletScene();
  }

  if(draw_bullet_result) {
    DrawBulletResult();
  }
#endif
}

void display() {
  Update();

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // set_orthoview_pass(width, height);

  switch(split_screen_vr_mode) {
  case 0: // cycloptic (no VR)
    display_for_one_eye(0, 0);
    break;
  case 1: // parallel viewing
    display_for_one_eye(1, eye_distance);
    display_for_one_eye(2, eye_distance);
    break;
  case 2: // cross-eyed viewing
    display_for_one_eye(1, -eye_distance);
    display_for_one_eye(2, -eye_distance);
    break;
  }

  glutSwapBuffers();
}

void reshape(int w, int h) {
  width = w;
  height = h;
}

void animate() {
  if (do_animate) {

    sub_frame++;
    if (sub_frame >= frame_step) {
      sub_frame = 0;
      current_frame++;
    }

    printf("\rFrame: %d", current_frame);
    fflush(stdout);
    // if (spinning) {
    //	add_quats(prev_quat, curr_quat, curr_quat);
    //}

  }

  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
#if defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA)
  if(state == GLUT_DOWN) {
    prev_mouse_coord.x = x;
    prev_mouse_coord.y = y;
    if(button == GLUT_LEFT_BUTTON) {
      left_mouse_down = true;
      prev_orient = orient;
    }
    if(button == GLUT_RIGHT_BUTTON) {
      right_mouse_down = true;
      prev_orbit_radius = orbit_radius;
    }
  }
  else {
    left_mouse_down = right_mouse_down = false;
  }
#else
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
#endif

  glutPostRedisplay();
}

void motion(int x, int y) {
#if defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA)
  if(left_mouse_down || right_mouse_down) {
    mouse_drag = glm::vec2(x, y) - prev_mouse_coord;
  }
  if(left_mouse_down) {
    orient = prev_orient+glm::vec3(0, mouse_drag.y*ORIENT_PITCH(orbit_speed), mouse_drag.x*ORIENT_YAW(orbit_speed));
    if(ORIENT_PITCH(orient) > MAX_PITCH) ORIENT_PITCH(orient) = MAX_PITCH;
    if(ORIENT_PITCH(orient) < MIN_PITCH) ORIENT_PITCH(orient) = MIN_PITCH;
    if(ORIENT_YAW(orient) > 360)         ORIENT_YAW(orient) -= 360;
    if(ORIENT_YAW(orient) < 0)           ORIENT_YAW(orient) += 360;
  }
  if(right_mouse_down) {
    orbit_radius = prev_orbit_radius + mouse_drag.y*dolly_speed;
    if(orbit_radius < 0) {
        orbit_radius = 0;
    }
  }
  if(left_mouse_down || right_mouse_down) {
    UpdateCameraParams();
  }
#else
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
#endif

  glutPostRedisplay();
}

void init();

void keyboard(unsigned char k, int x, int y) {
  bool redraw = false;
  bool rebuildBRDF = false;

  static int presetnum = 0;

  switch (k) {
  case 27: /* ESC */
  case 'q':
    exit(0);
    break;
  case 'a':
    do_animate = !do_animate;
    break;
  case 'x':
    draw_axis = !draw_axis;
    break;
  case 'k':
    draw_ik = !draw_ik;
    break;
  case 'm':
    draw_mesh = !draw_mesh;
    break;
  case 'b':
    draw_bbox_mode = (draw_bbox_mode+1)%4;
    break;
  case 'z':
    draw_bones = !draw_bones;
    break;
  case 'w':
    draw_mesh = true;
    draw_wireframe = !draw_wireframe;
    if(draw_wireframe) {
      glPolygonMode(GL_FRONT, GL_LINE);
    } else {
      glPolygonMode(GL_FRONT, GL_FILL);
    }
    break;
  case 'n':
    print_bone_info = !print_bone_info;
    break;
  case 'p':
    draw_bullet_scene = !draw_bullet_scene;
    break;
  case 'r':
    draw_bullet_result = !draw_bullet_result;
    break;
  case 'v':
    split_screen_vr_mode = (split_screen_vr_mode+1)%3;
    break;
  case ' ': /* space */
    /* reset view */
    init();
    break;

  default:
    break;
  }

  if (redraw) {
    display();
  }
}

static void IdentifyChainBones(std::string seed_name, std::set<std::string>* exception_list) {
  Bone* seed_bone = NULL;
  for(int i = 0; i < model->bones_.size(); i++) {
    if(model->bones_[i].ascii_name == seed_name) {
      seed_bone = &model->bones_[i];
      break;
    }
  }
  std::set<Bone*> flood_fill_bones;
  flood_fill_bones.insert(seed_bone);
  int n = 0;
  bool change = true;
  while(change) {
    change = false;
    for(int j = 0; j < model->bones_.size(); j++) {
      if((model->bones_[j].ascii_name.empty() ||
         (exception_list && exception_list->find(model->bones_[j].ascii_name) != exception_list->end())) &&
         flood_fill_bones.find(&model->bones_[model->bones_[j].parentIndex]) != flood_fill_bones.end())
      {
        std::stringstream ss;
        ss << seed_name << n;
        model->bones_[j].ascii_name = ss.str();
        flood_fill_bones.insert(&model->bones_[j]);
        change = true;
        n++;
      }
    }
  }
  flood_fill_bones.erase(seed_bone);
  for(int k = 0; k < model->bones_.size(); k++) {
    if(model->bones_[k].isChain || model->bones_[k].isPinnedChain) {
      continue;
    }

    // heuristic to identify chain bones:
    // if current bone is connected to seed bone directly/indirectly, current bone is a chain bone
    model->bones_[k].isChain = (flood_fill_bones.find(&model->bones_[k]) != flood_fill_bones.end());

    // heuristic to identify pinned chain bones:
    // if parent bone's tail index is illegal, parent bone must be a diverging bone
    // if parent bone is a diverging bone, current bone must be excluded from physics simulation
    // if parent bone is seed bone, current bone must be excluded from physics simulation
    model->bones_[k].isPinnedChain =
        (model->bones_[k].isChain && (!model->bones_[model->bones_[k].parentIndex].tailIndex ||
                                      &model->bones_[model->bones_[k].parentIndex] == seed_bone //||
                                      /*!model->bones_[k].hasVertices*/));

    // DBG
    //if(model->bones_[k].isPinnedChain) {
    //  std::cout << "PINNED_CHAIN: " << model->bones_[k].ascii_name << std::endl;
    //}
  }
}

void load(char *pmdmodel, char *vmdmodel) {
  PMDReader pmdreader;
  model = pmdreader.LoadFromFile(pmdmodel);
  assert(model);

  VMDReader vmdreader;
  anim = vmdreader.LoadFromFile(vmdmodel);
  assert(anim);

  scene = new MMDScene();
  scene->SetModel(model);
  scene->AttachAnimation(anim);

  renderVertices = new float[3 * model->vertices_.size()];

  DumpIK();
  DumpBone();

  CalculateBboxMinMax();

  IdentifyChainBones("head", NULL);

#ifdef ENABLE_EXTRA_PHYSICS
  // DBG (FIX-ME! -- unruly skirt/sleeves!)
  IdentifyChainBones("lower_body", NULL);
  std::set<std::string> exception_list;
  exception_list.insert("sleeve_L");
  exception_list.insert("sleeve_R");
  exception_list.insert("cuff_L");
  exception_list.insert("cuff_R");
  IdentifyChainBones("elbow_L", &exception_list);
  IdentifyChainBones("elbow_R", &exception_list);
#endif
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

  current_frame = 0;
  sub_frame = 0;

#if defined(ENABLE_GLM) && defined(ENABLE_EULER_CAMERA)
  prev_orient = orient = glm::vec3(0);
  orbit_radius = prev_orbit_radius =
      (scene->static_max.y - scene->static_min.y) * 0.5 * (1 / tan(glm::radians(view_fov * 0.5)));
  UpdateCameraParams();
#else
  view_org[0] = view_org[1] = 0.0;
  view_org[2] = 3.0;
  view_tgt[0] = view_tgt[1] = view_tgt[2] = 0.0;

  trackball(curr_quat, 0.0, 0.0, 0.0, 0.0);
#endif

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  do_animate = true;
  draw_axis = false;
  draw_ik = false;
  draw_mesh = true;
  draw_bbox_mode = 0;
  draw_bones = true;
  draw_wireframe = false;
  print_bone_info = false;
  draw_bullet_scene = false;
  split_screen_vr_mode = 0;

  srand(time(NULL));
}

int main(int argc, char **argv) {
  if (argc < 3) {
    printf("Usage: %s input.pmd input.vmd\n", argv[0]);
    exit(-1);
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

#ifdef ENABLE_BULLET
  InitSimulation();
#endif

  glutMainLoop();

#ifdef ENABLE_BULLET
  DeInitSimulation();
#endif

  return 0;
}
