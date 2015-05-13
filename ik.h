#ifndef IK_H_INCLUDED
#define IK_H_INCLUDED

#undef Success
#include "curveevaluator.h"
#include "modelerapp.h"
#include "Eigen/Dense"
#include <vector>

using namespace Eigen;


class body
{
    
public:
    body(){ mLen = 0; mAngles.push_back(0); mAngles.push_back(0);};
    body(float len, float angle_z, float angle_y){mLen = len; mAngles.push_back(angle_z); mAngles.push_back(angle_y);};
    
    float mLen;
    std::vector<float> mAngles;
    
};


class leg
{
    
public:
    leg(){ mLen = 0; mAngle = 0;};
    leg(float len, float angle_x){mLen = len; mAngle = angle_x;};
    
    float mLen;
    float mAngle;
    
};


class IK
{
public:
   
    IK(){
    
        mCurPosition = VectorXf::Zero(3, 1);
        mGoalPostion = VectorXf::Zero(3, 1);
        headDirction = VectorXf::Zero(3, 1);
        mArmStartPostion = VectorXf::Zero(3, 1);
        mPelPostion = VectorXf::Zero(3, 1);
        mFootPosition = VectorXf::Zero(3, 1);
        mRFootPosition = VectorXf::Zero(3, 1);
        mOffset = 0;
    
    };
    void start();
	
 
    VectorXf mCurPosition;
    VectorXf mGoalPostion;
    VectorXf mArmStartPostion;
    VectorXf mPelPostion;
    Vector3f headDirction;
    Vector3f mFootPosition;
    Vector3f mRFootPosition;
    float mOffset;
    
    MatrixXf jacobianInverse(float angle);
    MatrixXf walk_jacobianInverse(float angle, std::string lr);
    
    void calculateAngles(VectorXf dxdydz, float angle);
    void walk_calculateAngles(VectorXf dxdydz, float angle, std::string lr);
    
    bool getGoal();
    float rotateBody();
    void updateHeadDir(float angle);
    void updateArmPosition(Vector3f headDirction_var, Vector3f mPelPostion_var);
    void updateCurPosition(float anlge, Vector3f mArmStartPostion_var);
    void updateFootPosition(float anlge, std::string lr);
    void updateCurPositionInitial(Vector3f mArmStartPostion_var);
    float walk(ModelerApplication* app);
    float dp(float p1, float p2);
    bool isNearGoal(Vector3f headDirction_var, Vector3f mPelPostion_var);
    
    
    std::vector<body*> mBodys;
    std::vector<leg*> mLegs;
    

};

#endif // IK_H_INCLUDED
