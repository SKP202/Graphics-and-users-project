#include <GL/glut.h>
#include <cmath> // For C++ math functions
#include <GL/glu.h>
#include <stdio.h>  // For file I/O, printf
#include <stdlib.h> // For malloc, exit, rand, srand
#include <string.h> // For strcpy, strcmp
#include <time.h>   // For time (seeding rand)

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h" // For loading images as textures

// Defines for Mountain generation
#define NUM_SPLINE_CONTROL_POINTS 6 // Kept if spline logic is re-enabled later
#define MESH_SEGMENTS 40
#define MOUNTAIN_TEXTURE_SIZE 64

// === ROBOT DEFINITIONS ===
#define MAX_CHILDREN 10 // Max children per scene node
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

// Structure for a node in the scene graph (used for robot parts)
typedef struct SceneNode {
    char name[50];
    int isJoint; // 0 for body part (cube), 1 for joint (sphere), 2 for head (sphere)
    float angleX, angleY, angleZ; // Rotation relative to parent
    float posX, posY, posZ;       // Position relative to parent
    float sizeX, sizeY, sizeZ;    // Dimensions of the part
    float colorR, colorG, colorB; // Base color if not textured
    float ambient[4];             // Material properties
    float diffuse[4];
    float specular[4];
    float shininess;
    struct SceneNode* parent;
    struct SceneNode* children[MAX_CHILDREN];
    int childCount;
} SceneNode;

// Structure for the robot
typedef struct {
    SceneNode* root;
} Robot;

Robot robot;
int windowWidth = 800, windowHeight = 600;

// Default colors for robot parts
float defaultRobotColorR = 0.7f;
float defaultRobotColorG = 0.7f;
float defaultRobotColorB = 0.7f;
float defaultJointColorR = 0.5f;
float defaultJointColorG = 0.5f;
float defaultJointColorB = 0.5f;

// Robot Animation states & variables
enum AnimationState { ANIM_NONE, ANIM_WALKING, ANIM_WAVING };
AnimationState robotAnimationState = ANIM_NONE;

float walkAngle = 0.0f;
float waveAngle = 0.0f;

// Robot Path Following Variables
float robotPosX = 0.0f;
float robotPosY = 0.0f;
float robotPosZ = 0.0f;
float robotAngleY = 0.0f;
float targetRobotAngleY = 0.0f;
bool isRobotTurning = false;

// Robot movement and animation speeds
const float WALK_SPEED = 0.05f;
const float WALK_ANIM_SPEED = 1.5f;
const float ROBOT_TURN_SPEED = 4.0f;
const float WAVE_ANIM_SPEED = 2.0f;
const float ROBOT_SCALE = 0.5f;

// Square path definition for robot patrol around the house
const float PATH_CLEARANCE = 0.75f;
const float HOUSE_HALF_WIDTH = 1.0f;
const float HOUSE_HALF_DEPTH = 1.0f;

GLfloat pathPoints[4][2] = {
        { HOUSE_HALF_WIDTH + PATH_CLEARANCE,  HOUSE_HALF_DEPTH + PATH_CLEARANCE },
        { -(HOUSE_HALF_WIDTH + PATH_CLEARANCE), HOUSE_HALF_DEPTH + PATH_CLEARANCE },
        { -(HOUSE_HALF_WIDTH + PATH_CLEARANCE), -(HOUSE_HALF_DEPTH + PATH_CLEARANCE)},
        { HOUSE_HALF_WIDTH + PATH_CLEARANCE,  -(HOUSE_HALF_DEPTH + PATH_CLEARANCE)}
};
int currentPathTargetIndex = 0;


// === GLOBAL VARIABLES FOR SCENE ===
const float SCENE_Y_OFFSET = 2.0f;
const float GROUND_Y_LEVEL = -0.51f + SCENE_Y_OFFSET;

// Camera parameters
float camera_radius = 25.0f;
float camera_theta = 0.2f;
float camera_phi = 0.5f;
const float camera_look_at_base_X = 0.0f;
const float camera_look_at_base_Y = 0.5f;
const float camera_look_at_base_Z = 0.0f;

// Texture IDs
GLuint brickTexture = 0;
GLuint woodTexture = 0;
GLuint grassTexture = 0;
GLuint windowTexture = 0;
GLuint roofTexture = 0;
GLuint leafTexture = 0;
GLuint mountainPhotoTexture = 0;
GLuint robotMetalTexture = 0;

// Texture toggle flags
bool enableBrickTexture = true;
bool enableWoodTexture = true;
bool enableGrassTexture = true;
bool enableWindowTexture = true;
bool enableRoofTexture = true;
bool enableLeafTexture = true;
bool enableMountainPhotoTexture = true;
bool generalTexturesEnabled = true;
bool enableRobotTexture = true;

// Mountain generation and drawing parameters
GLfloat splineControlPoints[NUM_SPLINE_CONTROL_POINTS][3] = {
        {-8.0f, 0.0f, 0.0f}, {-4.0f, 2.0f, 0.0f}, { 0.0f, 4.0f, 0.0f},
        { 4.0f, 1.0f, 0.0f}, { 8.0f, 3.0f, 0.0f}, {12.0f, 0.0f, 0.0f}
};
GLuint hillTextureID_mountain;
float animatedHeightScale_mountain = 3.5f;
bool sceneLightingEnabled = true;

// Mountain display list and pre-calculated geometry
GLuint mountainDisplayListID;
GLfloat mountainVertices[MESH_SEGMENTS + 1][MESH_SEGMENTS + 1][3];
GLfloat mountainNormals[MESH_SEGMENTS + 1][MESH_SEGMENTS + 1][3];
GLfloat mountainTexCoords[MESH_SEGMENTS + 1][MESH_SEGMENTS + 1][2];

// Fence parameters
const float FENCE_POST_HEIGHT = 1.0f;
const float FENCE_POST_DIM = 0.1f;
const float FENCE_RAIL_THICKNESS = 0.05f;
const int FENCE_SECTIONS_PER_SIDE = 5;


// === FUNCTION PROTOTYPES ===
SceneNode* createSceneNode(const char* name, int isJoint, float x, float y, float z, float sx, float sy, float sz, float r, float g, float b);
void addChild(SceneNode* parent, SceneNode* child);
SceneNode* findNode(SceneNode* currentNode, const char* name);
void initRobot();
void drawNode(SceneNode* node);
void updateRobotState();
void drawTexturedRobotCube(float sx, float sy, float sz);
GLuint loadTexture(const char* filename);
void drawGround();
void drawCube(float x, float y, float z, float w, float h, float d, GLuint texture, bool enableFlag, float r, float g, float b);
void drawHouse();
void drawTree();
void drawFence();
void updateCamera();
void loadProceduralTexture_mountain();
void createMountainBaseGeometry(int segments);
void compileMountainDisplayList(int segments);
void drawMountain();
void init_combined();
void display_combined();
void reshape_combined(int w, int h);
void keyboard_combined(unsigned char key, int x, int y);
void mouse_combined(int button, int state, int x, int y);
void timer_combined(int value);

// === ROBOT FUNCTION IMPLEMENTATIONS ===
SceneNode* createSceneNode(const char* name, int isJointType, float x, float y, float z, float sx, float sy, float sz, float r, float g, float b) {
    SceneNode* node = (SceneNode*)malloc(sizeof(SceneNode));
    if (!node) { perror("Failed to allocate SceneNode"); exit(EXIT_FAILURE); }
    strcpy(node->name, name);
    node->isJoint = isJointType;
    node->angleX = node->angleY = node->angleZ = 0.0f;
    node->posX = x; node->posY = y; node->posZ = z;
    node->sizeX = sx; node->sizeY = sy; node->sizeZ = sz;
    node->colorR = r; node->colorG = g; node->colorB = b;
    node->parent = NULL; node->childCount = 0;
    node->ambient[0] = 0.2f * r; node->ambient[1] = 0.2f * g; node->ambient[2] = 0.2f * b; node->ambient[3] = 1.0f;
    node->diffuse[0] = 0.8f * r; node->diffuse[1] = 0.8f * g; node->diffuse[2] = 0.8f * b; node->diffuse[3] = 1.0f;
    node->specular[0] = 0.5f; node->specular[1] = 0.5f; node->specular[2] = 0.5f; node->specular[3] = 1.0f;
    node->shininess = 50.0f;
    return node;
}

void addChild(SceneNode* parent, SceneNode* child) {
    if (parent->childCount < MAX_CHILDREN) {
        parent->children[parent->childCount++] = child;
        child->parent = parent;
    } else {
        fprintf(stderr, "Error: Max children exceeded for node %s\n", parent->name);
    }
}

SceneNode* findNode(SceneNode* currentNode, const char* name) {
    if (!currentNode) return NULL;
    if (strcmp(currentNode->name, name) == 0) return currentNode;
    for (int i = 0; i < currentNode->childCount; ++i) {
        SceneNode* found = findNode(currentNode->children[i], name);
        if (found) return found;
    }
    return NULL;
}

void initRobot() {
    robot.root = createSceneNode("robot_root", 0, 0,0,0, 0,0,0, 0,0,0);
    SceneNode* torso = createSceneNode("torso", 0, 0, 0.75f, 0, 0.6f, 0.8f, 0.4f, defaultRobotColorR, defaultRobotColorG, defaultRobotColorB);
    addChild(robot.root, torso);
    SceneNode* head = createSceneNode("head", 2, 0, 0.6f, 0, 0.4f, 0.4f, 0.4f, defaultRobotColorR, defaultRobotColorG, defaultRobotColorB);
    addChild(torso, head);
    SceneNode* lShoulder = createSceneNode("left_shoulder", 1, 0.4f,0.3f,0, 0.2f,0.2f,0.2f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(torso, lShoulder);
    SceneNode* lUpperArm = createSceneNode("left_upper_arm",0, 0,-0.3f,0, 0.15f,0.4f,0.15f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(lShoulder, lUpperArm);
    SceneNode* lElbow = createSceneNode("left_elbow",1, 0,-0.25f,0, 0.18f,0.18f,0.18f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(lUpperArm, lElbow);
    SceneNode* lLowerArm = createSceneNode("left_lower_arm",0, 0,-0.25f,0, 0.12f,0.3f,0.12f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(lElbow, lLowerArm);
    SceneNode* rShoulder = createSceneNode("right_shoulder",1, -0.4f,0.3f,0, 0.2f,0.2f,0.2f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(torso, rShoulder);
    SceneNode* rUpperArm = createSceneNode("right_upper_arm",0, 0,-0.3f,0, 0.15f,0.4f,0.15f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(rShoulder, rUpperArm);
    SceneNode* rElbow = createSceneNode("right_elbow",1, 0,-0.25f,0, 0.18f,0.18f,0.18f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(rUpperArm, rElbow);
    SceneNode* rLowerArm = createSceneNode("right_lower_arm",0, 0,-0.25f,0, 0.12f,0.3f,0.12f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(rElbow, rLowerArm);
    SceneNode* lHip = createSceneNode("left_hip",1, 0.2f,-0.45f,0, 0.22f,0.22f,0.22f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(torso, lHip);
    SceneNode* lUpperLeg = createSceneNode("left_upper_leg",0, 0,-0.35f,0, 0.18f,0.5f,0.18f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(lHip, lUpperLeg);
    SceneNode* lKnee = createSceneNode("left_knee",1, 0,-0.3f,0, 0.2f,0.2f,0.2f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(lUpperLeg, lKnee);
    SceneNode* lLowerLeg = createSceneNode("left_lower_leg",0, 0,-0.3f,0, 0.15f,0.4f,0.15f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(lKnee, lLowerLeg);
    SceneNode* rHip = createSceneNode("right_hip",1, -0.2f,-0.45f,0, 0.22f,0.22f,0.22f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(torso, rHip);
    SceneNode* rUpperLeg = createSceneNode("right_upper_leg",0, 0,-0.35f,0, 0.18f,0.5f,0.18f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(rHip, rUpperLeg);
    SceneNode* rKnee = createSceneNode("right_knee",1, 0,-0.3f,0, 0.2f,0.2f,0.2f, defaultJointColorR,defaultJointColorG,defaultJointColorB); addChild(rUpperLeg, rKnee);
    SceneNode* rLowerLeg = createSceneNode("right_lower_leg",0, 0,-0.3f,0, 0.15f,0.4f,0.15f, defaultRobotColorR,defaultRobotColorG,defaultRobotColorB); addChild(rKnee, rLowerLeg);

    robotPosX = pathPoints[0][0];
    robotPosY = GROUND_Y_LEVEL - (-0.85f * ROBOT_SCALE);
    robotPosZ = pathPoints[0][1];
    currentPathTargetIndex = 1;

    float dx = pathPoints[currentPathTargetIndex][0] - robotPosX;
    float dz = pathPoints[currentPathTargetIndex][1] - robotPosZ;
    targetRobotAngleY = atan2(dx, dz) * 180.0f / M_PI;
    robotAngleY = targetRobotAngleY;
    isRobotTurning = false;
}

void drawTexturedRobotCube(float sx, float sy, float sz) {
    glPushMatrix();
    glScalef(sx, sy, sz);
    glBegin(GL_QUADS);
    // Front Face
    glNormal3f(0.0f, 0.0f, 1.0f); glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, 0.5f); glTexCoord2f(1.0f, 0.0f); glVertex3f( 0.5f, -0.5f, 0.5f); glTexCoord2f(1.0f, 1.0f); glVertex3f( 0.5f,  0.5f, 0.5f); glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f,  0.5f, 0.5f);
    // Back Face
    glNormal3f(0.0f, 0.0f, -1.0f); glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f); glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f,  0.5f, -0.5f); glTexCoord2f(0.0f, 1.0f); glVertex3f( 0.5f,  0.5f, -0.5f); glTexCoord2f(0.0f, 0.0f); glVertex3f( 0.5f, -0.5f, -0.5f);
    // Top Face
    glNormal3f(0.0f, 1.0f, 0.0f); glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f,  0.5f, -0.5f); glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f,  0.5f,  0.5f); glTexCoord2f(1.0f, 0.0f); glVertex3f( 0.5f,  0.5f,  0.5f); glTexCoord2f(1.0f, 1.0f); glVertex3f( 0.5f,  0.5f, -0.5f);
    // Bottom Face
    glNormal3f(0.0f, -1.0f, 0.0f); glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, -0.5f, -0.5f); glTexCoord2f(0.0f, 1.0f); glVertex3f( 0.5f, -0.5f, -0.5f); glTexCoord2f(0.0f, 0.0f); glVertex3f( 0.5f, -0.5f,  0.5f); glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f,  0.5f);
    // Right face
    glNormal3f(1.0f, 0.0f, 0.0f); glTexCoord2f(1.0f, 0.0f); glVertex3f( 0.5f, -0.5f, -0.5f); glTexCoord2f(1.0f, 1.0f); glVertex3f( 0.5f,  0.5f, -0.5f); glTexCoord2f(0.0f, 1.0f); glVertex3f( 0.5f,  0.5f,  0.5f); glTexCoord2f(0.0f, 0.0f); glVertex3f( 0.5f, -0.5f,  0.5f);
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f); glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f); glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f,  0.5f); glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f,  0.5f,  0.5f); glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f,  0.5f, -0.5f);
    glEnd();
    glPopMatrix();
}

void drawNode(SceneNode* node) {
    if (!node) return;
    glPushMatrix();
    glTranslatef(node->posX, node->posY, node->posZ);
    glRotatef(node->angleX, 1.0f, 0.0f, 0.0f);
    glRotatef(node->angleY, 0.0f, 1.0f, 0.0f);
    glRotatef(node->angleZ, 0.0f, 0.0f, 1.0f);
    bool useRobotTex = generalTexturesEnabled && enableRobotTexture && robotMetalTexture != 0;
    if (useRobotTex) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, robotMetalTexture);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D);
        glColor3f(node->colorR, node->colorG, node->colorB);
    }
    glMaterialfv(GL_FRONT, GL_AMBIENT, node->ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, node->diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, node->specular);
    glMaterialf(GL_FRONT, GL_SHININESS, node->shininess);
    if (node->sizeX > 0 && node->sizeY > 0 && node->sizeZ > 0) {
        if (node->isJoint == 1 || node->isJoint == 2) {
            GLUquadric* quad = gluNewQuadric();
            if (useRobotTex) {
                gluQuadricTexture(quad, GL_TRUE);
            } else {
                gluQuadricTexture(quad, GL_FALSE);
            }
            gluSphere(quad, node->sizeX / 2.0f, 16, 16);
            gluDeleteQuadric(quad);
        } else {
            if (useRobotTex) {
                drawTexturedRobotCube(node->sizeX, node->sizeY, node->sizeZ);
            } else {
                glPushMatrix();
                glScalef(node->sizeX, node->sizeY, node->sizeZ);
                glutSolidCube(1.0f);
                glPopMatrix();
            }
        }
    }
    if (useRobotTex) {
        glDisable(GL_TEXTURE_2D);
    }
    for (int i = 0; i < node->childCount; ++i) {
        drawNode(node->children[i]);
    }
    glPopMatrix();
}

void updateRobotState() {
    SceneNode* lShoulder = findNode(robot.root, "left_shoulder");
    SceneNode* rShoulder = findNode(robot.root, "right_shoulder");
    SceneNode* lHip = findNode(robot.root, "left_hip");
    SceneNode* rHip = findNode(robot.root, "right_hip");
    SceneNode* lElbow = findNode(robot.root, "left_elbow");
    SceneNode* rElbow = findNode(robot.root, "right_elbow");

    if (robotAnimationState == ANIM_WALKING || isRobotTurning) {
        walkAngle += WALK_ANIM_SPEED;
        float angleRad = walkAngle * M_PI / 180.0f;
        if (lHip) lHip->angleX = 30.0f * sin(angleRad);
        if (rHip) rHip->angleX = -30.0f * sin(angleRad);
        if (lShoulder) lShoulder->angleX = -30.0f * sin(angleRad);
        if (rShoulder) rShoulder->angleX = 30.0f * sin(angleRad);
    } else if (robotAnimationState == ANIM_WAVING) {
        waveAngle += WAVE_ANIM_SPEED;
        if (rShoulder) rShoulder->angleZ = 45.0f * sin(waveAngle * M_PI / 180.0f) - 30.0f;
        if (rElbow) rElbow->angleX = 30.0f * (1.0f + sin(waveAngle * M_PI / 180.0f + M_PI_2));
    } else {
        if (lHip) lHip->angleX = 0; if (rHip) rHip->angleX = 0;
        if (lShoulder) lShoulder->angleX = 0;
        if (rShoulder) { rShoulder->angleX = 0; rShoulder->angleZ = 0; }
        if (lElbow) lElbow->angleX = 0; if (rElbow) rElbow->angleX = 0;
    }

    if (robotAnimationState == ANIM_WALKING) {
        if (isRobotTurning) {
            float angleDiff = targetRobotAngleY - robotAngleY;
            while (angleDiff > 180.0f) angleDiff -= 360.0f;
            while (angleDiff < -180.0f) angleDiff += 360.0f;

            if (fabs(angleDiff) < ROBOT_TURN_SPEED) {
                robotAngleY = targetRobotAngleY;
                isRobotTurning = false;
            } else {
                robotAngleY += (angleDiff > 0 ? ROBOT_TURN_SPEED : -ROBOT_TURN_SPEED);
            }
            while (robotAngleY >= 360.0f) robotAngleY -= 360.0f;
            while (robotAngleY < 0.0f) robotAngleY += 360.0f;

        } else {
            float targetX = pathPoints[currentPathTargetIndex][0];
            float targetZ = pathPoints[currentPathTargetIndex][1];

            float dx = targetX - robotPosX;
            float dz = targetZ - robotPosZ;
            float distanceToTargetSq = dx * dx + dz * dz;

            if (distanceToTargetSq < WALK_SPEED * WALK_SPEED * 1.5f) {
                robotPosX = targetX;
                robotPosZ = targetZ;
                currentPathTargetIndex = (currentPathTargetIndex + 1) % 4;
                float nextTargetX = pathPoints[currentPathTargetIndex][0];
                float nextTargetZ = pathPoints[currentPathTargetIndex][1];
                float next_dx = nextTargetX - robotPosX;
                float next_dz = nextTargetZ - robotPosZ;
                targetRobotAngleY = atan2(next_dx, next_dz) * 180.0f / M_PI;
                isRobotTurning = true;
            } else {
                robotPosX += WALK_SPEED * sin(robotAngleY * M_PI / 180.0f);
                robotPosZ += WALK_SPEED * cos(robotAngleY * M_PI / 180.0f);
            }
        }
    }
}


// === SCENE FUNCTION IMPLEMENTATIONS ===
GLuint loadTexture(const char* filename) {
    int width, height, nrChannels;
    unsigned char *data = stbi_load(filename, &width, &height, &nrChannels, 0);
    GLuint textureID = 0;
    if (data) {
        GLenum format = GL_RGB, internalFormat = GL_RGB8;
        if (nrChannels == 1) { format = GL_RED; internalFormat = GL_RED; }
        else if (nrChannels == 3) { format = GL_RGB; internalFormat = GL_RGB8; }
        else if (nrChannels == 4) { format = GL_RGBA; internalFormat = GL_RGBA8; }
        else { fprintf(stderr, "Unknown #channels for %s\n", filename); stbi_image_free(data); return 0; }
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        if (gluBuild2DMipmaps(GL_TEXTURE_2D, internalFormat, width, height, format, GL_UNSIGNED_BYTE, data) != 0) {
            fprintf(stderr, "Mipmap error for %s: %s\n", filename, (char*)gluErrorString(glGetError()));
            glDeleteTextures(1, &textureID); textureID = 0;
        }
        stbi_image_free(data);
    } else { fprintf(stderr, "Failed to load texture: %s\n", filename); }
    glBindTexture(GL_TEXTURE_2D, 0);
    return textureID;
}

void drawGround() {
    bool useTexture = generalTexturesEnabled && enableGrassTexture && grassTexture != 0;
    if (useTexture) {
        glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, grassTexture); glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D); glColor3f(0.2f, 0.8f, 0.2f);
    }
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glTexCoord2f(0.0f,0.0f); glVertex3f(-15.0f, GROUND_Y_LEVEL, -15.0f);
    glTexCoord2f(15.0f,0.0f);glVertex3f( 15.0f, GROUND_Y_LEVEL, -15.0f);
    glTexCoord2f(15.0f,15.0f);glVertex3f( 15.0f, GROUND_Y_LEVEL,  15.0f);
    glTexCoord2f(0.0f,15.0f);glVertex3f(-15.0f, GROUND_Y_LEVEL,  15.0f);
    glEnd();
    if (useTexture) glDisable(GL_TEXTURE_2D);
}

void drawCube(float x, float y, float z, float w, float h, float d, GLuint texture, bool enableFlag, float r, float g, float b) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(w, h, d);
    bool useTexture = generalTexturesEnabled && enableFlag && texture != 0;
    if (useTexture) {
        glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, texture); glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D); glColor3f(r, g, b);
    }
    glBegin(GL_QUADS);
    glNormal3f(0,0,1); glTexCoord2f(0,0); glVertex3f(-0.5,-0.5,0.5); glTexCoord2f(1,0); glVertex3f(0.5,-0.5,0.5); glTexCoord2f(1,1); glVertex3f(0.5,0.5,0.5); glTexCoord2f(0,1); glVertex3f(-0.5,0.5,0.5);
    glNormal3f(0,0,-1); glTexCoord2f(1,0); glVertex3f(-0.5,-0.5,-0.5); glTexCoord2f(1,1); glVertex3f(-0.5,0.5,-0.5); glTexCoord2f(0,1); glVertex3f(0.5,0.5,-0.5); glTexCoord2f(0,0); glVertex3f(0.5,-0.5,-0.5);
    glNormal3f(0,1,0); glTexCoord2f(0,1); glVertex3f(-0.5,0.5,0.5); glTexCoord2f(0,0); glVertex3f(-0.5,0.5,-0.5); glTexCoord2f(1,0); glVertex3f(0.5,0.5,-0.5); glTexCoord2f(1,1); glVertex3f(0.5,0.5,0.5);
    glNormal3f(0,-1,0); glTexCoord2f(1,1); glVertex3f(-0.5,-0.5,0.5); glTexCoord2f(0,1); glVertex3f(0.5,-0.5,0.5); glTexCoord2f(0,0); glVertex3f(0.5,-0.5,-0.5); glTexCoord2f(1,0); glVertex3f(-0.5,-0.5,-0.5);
    glNormal3f(1,0,0); glTexCoord2f(1,0); glVertex3f(0.5,-0.5,0.5); glTexCoord2f(1,1); glVertex3f(0.5,0.5,0.5); glTexCoord2f(0,1); glVertex3f(0.5,0.5,-0.5); glTexCoord2f(0,0); glVertex3f(0.5,-0.5,-0.5);
    glNormal3f(-1,0,0); glTexCoord2f(0,0); glVertex3f(-0.5,-0.5,0.5); glTexCoord2f(1,0); glVertex3f(-0.5,-0.5,-0.5); glTexCoord2f(1,1); glVertex3f(-0.5,0.5,-0.5); glTexCoord2f(0,1); glVertex3f(-0.5,0.5,0.5);
    glEnd();
    if (useTexture) glDisable(GL_TEXTURE_2D);
    glPopMatrix();
}

void drawHouse() {
    drawCube(0, 0.5f, 0, 2.0f,1.0f,2.0f, brickTexture,enableBrickTexture, 0.7f,0.3f,0.2f);
    glPushMatrix();
    bool useRoofTex = generalTexturesEnabled && enableRoofTexture && roofTexture!=0;
    if(useRoofTex){glEnable(GL_TEXTURE_2D);glBindTexture(GL_TEXTURE_2D,roofTexture);glColor3f(1,1,1);}
    else{glDisable(GL_TEXTURE_2D);glColor3f(0.8f,0.2f,0.2f);}
    glTranslatef(0, 1.0f, 0);
    glBegin(GL_TRIANGLES);
    glNormal3f(0,0.707f,0.707f);glTexCoord2f(0.5f,1);glVertex3f(0,0.8f,0);glTexCoord2f(0,0);glVertex3f(-1.1f,0,1.1f);glTexCoord2f(1,0);glVertex3f(1.1f,0,1.1f);
    glNormal3f(0,0.707f,-0.707f);glTexCoord2f(0.5f,1);glVertex3f(0,0.8f,0);glTexCoord2f(0,0);glVertex3f(1.1f,0,-1.1f);glTexCoord2f(1,0);glVertex3f(-1.1f,0,-1.1f);
    glNormal3f(-0.707f,0.707f,0);glTexCoord2f(0.5f,1);glVertex3f(0,0.8f,0);glTexCoord2f(0,0);glVertex3f(-1.1f,0,-1.1f);glTexCoord2f(1,0);glVertex3f(-1.1f,0,1.1f);
    glNormal3f(0.707f,0.707f,0);glTexCoord2f(0.5f,1);glVertex3f(0,0.8f,0);glTexCoord2f(0,0);glVertex3f(1.1f,0,1.1f);glTexCoord2f(1,0);glVertex3f(1.1f,0,-1.1f);
    glEnd();
    if(useRoofTex)glDisable(GL_TEXTURE_2D);
    glPopMatrix();
    drawCube(0, 0.25f, 1.01f, 0.35f,0.5f,0.1f, woodTexture,enableWoodTexture,0.5f,0.35f,0.05f);
    drawCube(-0.6f, 0.5f + 0.2f, 1.01f, 0.3f,0.3f,0.1f, windowTexture,enableWindowTexture,0.7f,0.8f,0.9f);
    drawCube(0.6f, 0.5f + 0.2f, 1.01f, 0.3f,0.3f,0.1f, windowTexture,enableWindowTexture,0.7f,0.8f,0.9f);
    drawCube(0.6f, 1.0f + 0.25f, -0.5f, 0.3f,0.5f,0.3f, brickTexture,enableBrickTexture,0.6f,0.25f,0.15f);
}

void drawTree() {
    drawCube(0, 0.5f, 0, 0.2f,1.0f,0.2f, woodTexture,enableWoodTexture,0.4f,0.25f,0.0f);
    glPushMatrix();
    glTranslatef(0, 1.0f + 0.25f, 0);
    bool useLeafTex = generalTexturesEnabled && enableLeafTexture && leafTexture!=0;
    if(useLeafTex){glEnable(GL_TEXTURE_2D);glBindTexture(GL_TEXTURE_2D,leafTexture);glColor3f(1,1,1);GLUquadric* q=gluNewQuadric();gluQuadricTexture(q,GL_TRUE);gluSphere(q,0.6,16,16);gluDeleteQuadric(q);glDisable(GL_TEXTURE_2D);}
    else{glDisable(GL_TEXTURE_2D);glColor3f(0,0.5f,0);glutSolidSphere(0.6,16,16);}
    glPopMatrix();
}

void drawFence() {
    float fence_extent = 2.0f * (HOUSE_HALF_WIDTH + PATH_CLEARANCE);
    float post_y = GROUND_Y_LEVEL;
    float section_length = (2.0f * fence_extent) / FENCE_SECTIONS_PER_SIDE;
    bool useFenceTexture = generalTexturesEnabled && enableWoodTexture && woodTexture != 0;
    float rail1_y = GROUND_Y_LEVEL + (FENCE_POST_HEIGHT / 2.0f) * 0.3f;
    float rail2_y = GROUND_Y_LEVEL + (FENCE_POST_HEIGHT / 2.0f) * 0.7f;

    for (int fb = 0; fb < 2; ++fb) {
        float current_z = (fb == 0) ? fence_extent : -fence_extent;
        for (int i = 0; i <= FENCE_SECTIONS_PER_SIDE; ++i) {
            float current_x = -fence_extent + i * section_length;
            drawCube(current_x, post_y, current_z, FENCE_POST_DIM, FENCE_POST_HEIGHT, FENCE_POST_DIM, woodTexture, useFenceTexture, 0.4f, 0.25f, 0.1f);
            if (i < FENCE_SECTIONS_PER_SIDE) {
                float next_x = -fence_extent + (i + 1) * section_length;
                float rail_center_x = (current_x + next_x) / 2.0f;
                drawCube(rail_center_x, rail1_y, current_z, section_length, FENCE_RAIL_THICKNESS, FENCE_RAIL_THICKNESS, woodTexture, useFenceTexture, 0.5f, 0.3f, 0.15f);
                drawCube(rail_center_x, rail2_y, current_z, section_length, FENCE_RAIL_THICKNESS, FENCE_RAIL_THICKNESS, woodTexture, useFenceTexture, 0.5f, 0.3f, 0.15f);
            }
        }
    }
    for (int lr = 0; lr < 2; ++lr) {
        float current_x = (lr == 0) ? fence_extent : -fence_extent;
        for (int i = 0; i <= FENCE_SECTIONS_PER_SIDE; ++i) {
            float current_z = -fence_extent + i * section_length;
            drawCube(current_x, post_y, current_z, FENCE_POST_DIM, FENCE_POST_HEIGHT, FENCE_POST_DIM, woodTexture, useFenceTexture, 0.4f, 0.25f, 0.1f);
            if (i < FENCE_SECTIONS_PER_SIDE) {
                float next_z = -fence_extent + (i + 1) * section_length;
                float rail_center_z = (current_z + next_z) / 2.0f;
                drawCube(current_x, rail1_y, rail_center_z, FENCE_RAIL_THICKNESS, FENCE_RAIL_THICKNESS, section_length, woodTexture, useFenceTexture, 0.5f, 0.3f, 0.15f);
                drawCube(current_x, rail2_y, rail_center_z, FENCE_RAIL_THICKNESS, FENCE_RAIL_THICKNESS, section_length, woodTexture, useFenceTexture, 0.5f, 0.3f, 0.15f);
            }
        }
    }
}


void updateCamera() {
    float actual_cam_target_Y = camera_look_at_base_Y + SCENE_Y_OFFSET;
    float camX = camera_look_at_base_X + camera_radius * sin(camera_theta) * cos(camera_phi);
    float camY = actual_cam_target_Y + camera_radius * sin(camera_phi);
    float camZ = camera_look_at_base_Z + camera_radius * cos(camera_theta) * cos(camera_phi);
    glLoadIdentity();
    gluLookAt(camX,camY,camZ, camera_look_at_base_X,actual_cam_target_Y,camera_look_at_base_Z, 0,1,0);
}

void createMountainBaseGeometry(int segments) {
    float terrain_scale_x = 20.0f;
    float terrain_scale_z = 20.0f;
    float base_height_scale = 1.0f;

    for (int i_idx = 0; i_idx <= segments; i_idx++) {
        for (int j_idx = 0; j_idx <= segments; j_idx++) {
            float x_norm = (float)i_idx / segments * 2.0f - 1.0f;
            float z_norm = (float)j_idx / segments * 2.0f - 1.0f;
            float y_base_val = exp(-(x_norm * x_norm * 2.0f + z_norm * z_norm * 2.0f)) * 2.5f +
                               exp(-((x_norm - 0.6f) * (x_norm - 0.6f) * 5.0f + (z_norm + 0.5f) * (z_norm + 0.5f) * 5.0f)) * 1.5f +
                               sin(x_norm * M_PI * 3.0f + z_norm * M_PI * 1.0f) * 0.3f +
                               (rand() / (float)RAND_MAX - 0.5f) * 0.1f;
            float y_final_val = fmax(0.05f * base_height_scale, y_base_val * base_height_scale);

            mountainVertices[i_idx][j_idx][0] = x_norm * terrain_scale_x;
            mountainVertices[i_idx][j_idx][1] = y_final_val - 0.5f * base_height_scale;
            mountainVertices[i_idx][j_idx][2] = z_norm * terrain_scale_z;

            mountainTexCoords[i_idx][j_idx][0] = (float)i_idx / segments * 4.0f;
            mountainTexCoords[i_idx][j_idx][1] = (float)j_idx / segments * 4.0f;
        }
    }
    for (int i_idx = 0; i_idx <= segments; i_idx++) {
        for (int j_idx = 0; j_idx <= segments; j_idx++) {
            float v_left[3], v_right[3], v_down[3], v_up[3];
            for (int k_dim = 0; k_dim < 3; k_dim++) {
                v_left[k_dim] = mountainVertices[i_idx > 0 ? i_idx - 1 : i_idx][j_idx][k_dim];
                v_right[k_dim] = mountainVertices[i_idx < segments ? i_idx + 1 : i_idx][j_idx][k_dim];
                v_down[k_dim] = mountainVertices[i_idx][j_idx > 0 ? j_idx - 1 : j_idx][k_dim];
                v_up[k_dim] = mountainVertices[i_idx][j_idx < segments ? j_idx + 1 : j_idx][k_dim];
            }
            float tan_u_vec[3] = { v_right[0] - v_left[0], v_right[1] - v_left[1], v_right[2] - v_left[2] };
            float tan_v_vec[3] = { v_up[0] - v_down[0], v_up[1] - v_down[1], v_up[2] - v_down[2] };
            float normal_vec[3] = { tan_v_vec[1] * tan_u_vec[2] - tan_v_vec[2] * tan_u_vec[1],
                                    tan_v_vec[2] * tan_u_vec[0] - tan_v_vec[0] * tan_u_vec[2],
                                    tan_v_vec[0] * tan_u_vec[1] - tan_v_vec[1] * tan_u_vec[0] };
            float len_normal = sqrt(normal_vec[0] * normal_vec[0] + normal_vec[1] * normal_vec[1] + normal_vec[2] * normal_vec[2]);
            if (len_normal > 1e-6) {
                mountainNormals[i_idx][j_idx][0] = normal_vec[0] / len_normal;
                mountainNormals[i_idx][j_idx][1] = normal_vec[1] / len_normal;
                mountainNormals[i_idx][j_idx][2] = normal_vec[2] / len_normal;
            } else {
                mountainNormals[i_idx][j_idx][0] = 0; mountainNormals[i_idx][j_idx][1] = 1; mountainNormals[i_idx][j_idx][2] = 0;
            }
        }
    }
}

void compileMountainDisplayList(int segments) {
    mountainDisplayListID = glGenLists(1);
    glNewList(mountainDisplayListID, GL_COMPILE);
    for (int i_idx = 0; i_idx < segments; i_idx++) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int j_idx = 0; j_idx <= segments; j_idx++) {
            glNormal3fv(mountainNormals[i_idx][j_idx]);
            glTexCoord2fv(mountainTexCoords[i_idx][j_idx]);
            glVertex3fv(mountainVertices[i_idx][j_idx]);

            glNormal3fv(mountainNormals[i_idx + 1][j_idx]);
            glTexCoord2fv(mountainTexCoords[i_idx + 1][j_idx]);
            glVertex3fv(mountainVertices[i_idx + 1][j_idx]);
        }
        glEnd();
    }
    glEndList();
}

void drawMountain() {
    bool usePhotoTex = generalTexturesEnabled && enableMountainPhotoTexture && mountainPhotoTexture != 0;
    bool useProceduralTex = !usePhotoTex && generalTexturesEnabled && hillTextureID_mountain != 0;

    if (usePhotoTex) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, mountainPhotoTexture);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else if (useProceduralTex) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, hillTextureID_mountain);
        glColor3f(1.0f, 1.0f, 1.0f);
    } else {
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.4f, 0.35f, 0.3f);
    }

    GLfloat hill_specular_mat[] = { 0.05f, 0.05f, 0.05f, 1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, hill_specular_mat);
    glMaterialf(GL_FRONT, GL_SHININESS, 5.0f);

    glPushMatrix();
    glScalef(1.0f, animatedHeightScale_mountain, 1.0f);
    glCallList(mountainDisplayListID);
    glPopMatrix();

    if (usePhotoTex || useProceduralTex) {
        glDisable(GL_TEXTURE_2D);
    }

    GLfloat default_specular_mat[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, default_specular_mat);
    glMaterialf(GL_FRONT, GL_SHININESS, 10.0f);
}


void loadProceduralTexture_mountain() {
    unsigned char imageData[MOUNTAIN_TEXTURE_SIZE*MOUNTAIN_TEXTURE_SIZE*3];
    for(int y_tex=0; y_tex<MOUNTAIN_TEXTURE_SIZE; y_tex++) {
        for(int x_tex=0; x_tex<MOUNTAIN_TEXTURE_SIZE; x_tex++) {
            int idx=(y_tex*MOUNTAIN_TEXTURE_SIZE+x_tex)*3;
            unsigned char r_val,g_val,b_val;
            char c1r_base=60+(rand()%20),c1g_base=80+(rand()%30),c1b_base=40+(rand()%10);
            char c2r_base=100+(rand()%20),c2g_base=120+(rand()%30),c2b_base=60+(rand()%10);
            if((x_tex/8+y_tex/8)%2==0){
                r_val=c1r_base+(x_tex%8)*2-(y_tex%4);g_val=c1g_base+(y_tex%8)*2-(x_tex%4);b_val=c1b_base;
            } else {
                r_val=c2r_base-(x_tex%8)*2+(y_tex%4);g_val=c2g_base-(y_tex%8)*2+(x_tex%4);b_val=c2b_base;
            }
            imageData[idx]=r_val; imageData[idx+1]=g_val; imageData[idx+2]=b_val;
        }
    }
    glGenTextures(1,&hillTextureID_mountain);glBindTexture(GL_TEXTURE_2D,hillTextureID_mountain);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,MOUNTAIN_TEXTURE_SIZE,MOUNTAIN_TEXTURE_SIZE,0,GL_RGB,GL_UNSIGNED_BYTE,imageData);
    glBindTexture(GL_TEXTURE_2D,0);
}


void init_combined() {
    glClearColor(0.53f,0.81f,0.98f,1.0f);glEnable(GL_DEPTH_TEST);glShadeModel(GL_SMOOTH);glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);glEnable(GL_COLOR_MATERIAL);glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    GLfloat light_pos0[]={10.0f,15.0f,10.0f,1.0f},amb0[]={0.3f,0.3f,0.3f,1.0f},diff0[]={0.9f,0.9f,0.8f,1.0f},spec0[]={0.5f,0.5f,0.5f,1.0f};
    glLightfv(GL_LIGHT0,GL_POSITION,light_pos0);glLightfv(GL_LIGHT0,GL_AMBIENT,amb0);glLightfv(GL_LIGHT0,GL_DIFFUSE,diff0);glLightfv(GL_LIGHT0,GL_SPECULAR,spec0);glEnable(GL_LIGHT0);
    GLfloat light_pos1[]={-10.0f,5.0f,-5.0f,1.0f},diff1[]={0.3f,0.3f,0.35f,1.0f};
    glLightfv(GL_LIGHT1,GL_POSITION,light_pos1);glLightfv(GL_LIGHT1,GL_DIFFUSE,diff1);glEnable(GL_LIGHT1);
    GLfloat global_amb[]={0.2f,0.2f,0.25f,1.0f};glLightModelfv(GL_LIGHT_MODEL_AMBIENT,global_amb);glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
    GLfloat mat_spec_default[]={0.1f,0.1f,0.1f,1.0f},mat_shine_default[]={10.0f};
    glMaterialfv(GL_FRONT,GL_SPECULAR,mat_spec_default);glMaterialfv(GL_FRONT,GL_SHININESS,mat_shine_default);
    stbi_set_flip_vertically_on_load(true);

    brickTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\brick.png)");
    woodTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\wood.png)");
    grassTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\grass.png)");
    windowTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\window.png)");
    roofTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\roof.png)");
    leafTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\leaves.png)");
    mountainPhotoTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\mountain.png)");
    robotMetalTexture = loadTexture(R"(C:\Users\feche\Desktop\Images\metal.png)");

    if(!brickTexture||!woodTexture||!grassTexture||!windowTexture||!roofTexture||!leafTexture||!mountainPhotoTexture)fprintf(stderr,"Warning: One or more scene textures failed to load. Check paths.\n");
    if(!robotMetalTexture) fprintf(stderr, "Warning: Robot metal texture (metal.png) failed to load. Check path.\n");

    loadProceduralTexture_mountain();
    if(!hillTextureID_mountain)fprintf(stderr,"Warning: Procedural mountain texture failed.\n");

    createMountainBaseGeometry(MESH_SEGMENTS);
    compileMountainDisplayList(MESH_SEGMENTS);

    initRobot();
}

void display_combined() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);glMatrixMode(GL_MODELVIEW);updateCamera();
    if(sceneLightingEnabled)glEnable(GL_LIGHTING);else glDisable(GL_LIGHTING);

    drawGround();

    glPushMatrix();
    glTranslatef(0.0f, GROUND_Y_LEVEL, 0.0f);
    drawHouse();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-2.5f, GROUND_Y_LEVEL, -1.0f);
    drawTree();
    glPopMatrix();

    drawFence();

    glPushMatrix();
    glTranslatef(0.0f,0.0f,-25.0f);
    drawMountain();
    glPopMatrix();

    if(robot.root){
        glPushMatrix();
        glTranslatef(robotPosX,robotPosY,robotPosZ);
        glRotatef(robotAngleY,0.0f,1.0f,0.0f);
        glScalef(ROBOT_SCALE, ROBOT_SCALE, ROBOT_SCALE);
        drawNode(robot.root);
        glPopMatrix();
    }
    glutSwapBuffers();
}

void reshape_combined(int w,int h){
    windowWidth=w;windowHeight=h;if(h==0)h=1;glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);glLoadIdentity();gluPerspective(45.0,(float)w/h,0.1,200.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard_combined(unsigned char key,int x_coord,int y_coord){
    float cam_move_speed=0.05f;
    switch(key){
        case 'a':case 'A':camera_theta-=cam_move_speed;break;
        case 'd':case 'D':camera_theta+=cam_move_speed;break;
        case 'w':case 'W':camera_phi+=cam_move_speed;break;
        case 's':case 'S':camera_phi-=cam_move_speed;break;
        case 't':case 'T':generalTexturesEnabled=!generalTexturesEnabled;break;
        case '1':enableBrickTexture=!enableBrickTexture;break;
        case '2':enableWoodTexture=!enableWoodTexture;break;
        case '3':enableGrassTexture=!enableGrassTexture;break;
        case '4':enableWindowTexture=!enableWindowTexture;break;
        case '5':enableRoofTexture=!enableRoofTexture;break;
        case '6':enableLeafTexture=!enableLeafTexture;break;
        case '7':enableMountainPhotoTexture=!enableMountainPhotoTexture;break;
        case '8':enableRobotTexture = !enableRobotTexture; printf("Robot Texture: %s\n", enableRobotTexture ? "ON" : "OFF"); break;
        case 'l':case 'L':sceneLightingEnabled=!sceneLightingEnabled;break;
        case 'p':case 'P':
            if(robotAnimationState==ANIM_WALKING){
                robotAnimationState=ANIM_NONE;
                printf("Robot: Patrol stopped.\n");
            } else {
                robotAnimationState=ANIM_WALKING;
                robotPosX = pathPoints[0][0];
                robotPosZ = pathPoints[0][1];
                robotPosY = GROUND_Y_LEVEL - (-0.85f * ROBOT_SCALE);
                currentPathTargetIndex = 1;
                float dx_init = pathPoints[currentPathTargetIndex][0] - robotPosX;
                float dz_init = pathPoints[currentPathTargetIndex][1] - robotPosZ;
                targetRobotAngleY = atan2(dx_init, dz_init) * 180.0f / M_PI;
                robotAngleY = targetRobotAngleY;
                isRobotTurning = false;
                printf("Robot: Patrol started.\n");
            }
            break;
        case 'o':case 'O':
            if(robotAnimationState==ANIM_WAVING){
                robotAnimationState=ANIM_NONE;
                printf("Robot: Waving stopped.\n");
            } else {
                robotAnimationState=ANIM_WAVING;
                printf("Robot: Waving started.\n");
            }
            break;
        case 27:exit(0);break;
    }
    if(camera_phi > M_PI_2 - 0.01f) camera_phi = M_PI_2 - 0.01f;
    if(camera_phi < -M_PI_2 + 0.01f) camera_phi = -M_PI_2 + 0.01f;
}

void mouse_combined(int button,int state,int x_mouse,int y_mouse){
    float zoom_factor=0.5f;
    if(state==GLUT_UP){
        if(button==3){camera_radius-=zoom_factor;if(camera_radius<1.0f)camera_radius=1.0f;}
        else if(button==4){camera_radius+=zoom_factor;if(camera_radius>100.0f)camera_radius=100.0f;}
    }
}

void timer_combined(int value){
    updateRobotState();
    glutPostRedisplay();
    glutTimerFunc(16,timer_combined,0);
}

int main(int argc,char** argv){
    srand(time(NULL));
    glutInit(&argc,argv);glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
    glutInitWindowSize(windowWidth,windowHeight);glutCreateWindow("House, Mountain, Robot Patrol");
    init_combined();
    glutDisplayFunc(display_combined);glutReshapeFunc(reshape_combined);
    glutKeyboardFunc(keyboard_combined);glutMouseFunc(mouse_combined);
    glutTimerFunc(0,timer_combined,0);
    printf("Scene Controls: W/S/A/D (Cam Rotate), Mouse Scroll (Zoom), T (All Tex), 1-7 (Obj Tex), 8 (Robot Tex), L (Light)\n");
    printf("Robot Controls: P (Toggle Patrol), O (Toggle Wave)\nESC to Exit.\n");
    printf("Path points (X,Z): P0(%.1f,%.1f), P1(%.1f,%.1f), P2(%.1f,%.1f), P3(%.1f,%.1f)\n",
           pathPoints[0][0], pathPoints[0][1], pathPoints[1][0], pathPoints[1][1],
           pathPoints[2][0], pathPoints[2][1], pathPoints[3][0], pathPoints[3][1]);
    glutMainLoop();
    return 0;
}
