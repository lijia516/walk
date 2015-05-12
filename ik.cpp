#include "ik.h"

#include <algorithm>
#include "modelerview.h"
#include "modelerdraw.h"
#include "graphwidget.h"
#include "animatoruiwindows.h"
#include "modelerapp.h"	// needed to read values from graph widget
#include "modelerui.h"	// needed to read values from graph widget


#include "mat.h"
#include <FL/gl.h>
#include <cstdlib>
#include <cassert>
#include <cmath>

#include <FL/gl.h>
#include <cstdlib>

using namespace std;

float PI = M_PI;

int jointNum = 2;


void IK::start(){
    
    ModelerApplication* app;
    int total_curves;	// This is the total number of items that appear in the
    // "model controls" pane, and equals the number of curves
    // for the model + number of curves for the camera.
    
    app = ModelerApplication::Instance();
    // The camera curves are always last in the list of "model curves" in the left pane.
    //total_curves = app->GetUI()->m_pwndGraphWidget->numCurves();
    //app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, 5,50);
    
    
 //   std::cout <<"goal:" << mGoalPostion(0) << "," << mGoalPostion(1) << "," << mGoalPostion(2)<< "\n";
 //   std::cout <<"cur:" << mCurPosition(0) << "," << mCurPosition(1) << "," << mCurPosition(2) << "\n";
    
 //   std::cout <<"armstart:" << mArmStartPostion(0) << "," << mArmStartPostion(1) << "," << mArmStartPostion(2) << "\n";
 //   std::cout <<"pel:" << mPelPostion(0) << "," << mPelPostion(1) << "," << mPelPostion(2) << "\n";

 //   std::cout <<"mFootPostion:" << mFootPosition(0) << "," << mFootPosition(1) << "," << mFootPosition(2) << "\n";
    
 //   std::cout <<"mRFootPostion:" << mRFootPosition(0) << "," << mRFootPosition(1) << "," << mRFootPosition(2) << "\n";
    
    
    float dt = 0.3;
    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(0);
    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(23);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(22);
    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(32);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(31);
    
     app->GetUI()->m_pwndGraphWidget->AddCtrlPt(0, 0, 0);
    
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, 0, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(22, 0, 0);
    
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, 0, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(31, 0, 0);
    
    
    // check if need to rotate body
    
    
    float angle = 0;
    
     float distance = sqrt(pow(mGoalPostion(0) - mArmStartPostion(0), 2) + pow(mGoalPostion(1) - mArmStartPostion(1), 2) + pow(mGoalPostion(2) - mArmStartPostion(2), 2));
    
    if (distance > 1.6) {
    
        angle = rotateBody();
    
        if (angle != 0) {
        
            updateHeadDir(angle);
            updateArmPosition(headDirction, mPelPostion);
            updateCurPositionInitial(mArmStartPostion);
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(0, dt, angle * 180.0 / PI);
        
        }
    
    }
    
    cout << "distance" << distance<<" \n";
    
    if ( distance > 1.6) {
        
        cout << "need to walk \n";
        
        dt = walk(app);
        
    }
    
  //  std::cout <<"pel:" << mPelPostion(0) << "," << mPelPostion(1) << "," << mPelPostion(2) << "\n";
    
    
    mPelPostion(0) = mOffset + fabs(mPelPostion(2)) * headDirction(0);
    mPelPostion(2) = fabs(mPelPostion(2)) * headDirction(2);

    
  //  std::cout <<"pel:" << mPelPostion(0) << "," << mPelPostion(1) << "," << mPelPostion(2) << "\n";
    

    
    updateArmPosition(headDirction, mPelPostion);
    updateCurPositionInitial(mArmStartPostion);

    
    
    float dx;
    float dy;
    float dz;
    
    dx = (mGoalPostion(0) - mCurPosition(0) > 0.1) ? 0.1 : -0.1;
    dy = (mGoalPostion(1) - mCurPosition(1) > 0.1) ? 0.1 : -0.1;
    dz = (mGoalPostion(2) - mCurPosition(2) > 0.1) ? 0.1 : -0.1;
    
    
  //  std::cout <<"dx, dy, dz:" << dx << "," << dy << "," << dz << "\n";

    
    
    while (!getGoal() && dt < 19) {
        
        dt += 0.1;
        
        std::cout <<"dt: "<< dt <<"\n";
        
        VectorXf dxdy = VectorXf::Zero(3, 1);
        dxdy(0) = dx;
        dxdy(1) = dy;
        dxdy(2) = dz;
        
        calculateAngles(dxdy, -angle);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, dt, mBodys[0]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(22, dt, mBodys[0]->mAngles[1] * 180.0 / PI);
        
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, dt, mBodys[1]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(31, dt, mBodys[1]->mAngles[1] * 180.0 / PI);
        
        
        dx = dp (mGoalPostion(0), mCurPosition(0));
        dy = dp (mGoalPostion(1), mCurPosition(1));
        dz = dp (mGoalPostion(2), mCurPosition(2));
        
     //   std::cout <<"ls: angle1, angle2:" << mBodys[0]->mAngles[0] << "," << mBodys[0]->mAngles[1]<<"\n";
     //   std::cout <<"lf: angle1, angle2:" << mBodys[1]->mAngles[0] << "," << mBodys[1]->mAngles[1]<<"\n";
        
        
    }
    
    ParticleSystem::start_time = dt;
    
}



MatrixXf IK::jacobianInverse(float angle) {
    
    
    MatrixXf jacobianMatrx = MatrixXf::Zero(3, mBodys.size() * 2);
    
    
    for (int j = 0; j < mBodys.size(); j++) {
        
        float angle_z = 0;
        float angle_y = 0;
        
        for (int i = 0; i < mBodys.size(); i++) {
            
            angle_z += mBodys[i]->mAngles[0];
            angle_y += mBodys[i]->mAngles[1];
            
            
            if (i >= j) {
                
             //   jacobianMatrx(0, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * cos(angle_y));
             //   jacobianMatrx(1, j*2) += (float)(mBodys[i]->mLen * sin(angle_z));
             //   jacobianMatrx(2, j*2) += (float)(- mBodys[i]->mLen * cos(angle_z) * sin(angle_y));
                
                
                jacobianMatrx(0, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * (cos(angle) * cos(angle_y) + sin(angle) * sin(angle_y)));
                                                 
                jacobianMatrx(1, j*2) += (float)(mBodys[i]->mLen * sin(angle_z));
                                                 
                jacobianMatrx(2, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * (sin(angle) * cos(angle_y) - cos(angle) * sin(angle_y)));
                
                
            //    jacobianMatrx(0, j*2+1) += (float)(-mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
            //    jacobianMatrx(1, j*2+1) += (float)0.0f;
            //    jacobianMatrx(2, j*2+1) += (float)(- mBodys[i]->mLen * sin(angle_z) * cos(angle_y));
            
                                                 
                jacobianMatrx(0, j*2+1) += (float) (mBodys[i]->mLen * sin(angle_z) * (sin(angle) * cos(angle_y) - cos(angle) * sin(angle_y)));
                
                jacobianMatrx(1, j*2+1) += (float)0.0f;
                
                jacobianMatrx(2, j*2+1) += (float) (-mBodys[i]->mLen * sin(angle_z) * (cos(angle) * cos(angle_y) + sin(angle) * sin(angle_y)));
                
            }
                                                 
                                                 
        }
    }

    
    MatrixXf jacobianMatrxInverse1 = MatrixXf::Zero(3,3);
    
    jacobianMatrxInverse1 = jacobianMatrx * (jacobianMatrx.transpose());
    
    
    float m00 = roundf(jacobianMatrxInverse1(0,0) * 1000000) / 1000000.0;
    float m01 = roundf(jacobianMatrxInverse1(0,1) * 1000000) / 1000000.0;
    float m02 = roundf(jacobianMatrxInverse1(0,2) * 1000000) / 1000000.0;
    float m10 = roundf(jacobianMatrxInverse1(1,0) * 1000000) / 1000000.0;
    float m11 = roundf(jacobianMatrxInverse1(1,1) * 1000000) / 1000000.0;
    float m12 = roundf(jacobianMatrxInverse1(1,2) * 1000000) / 1000000.0;
    float m20 = roundf(jacobianMatrxInverse1(2,0) * 1000000) / 1000000.0;
    float m21 = roundf(jacobianMatrxInverse1(2,1) * 1000000) / 1000000.0;
    float m22 = roundf(jacobianMatrxInverse1(2,2) * 1000000) / 1000000.0;
    
    Matrix3f jacobianMatrxInverse;
    
    jacobianMatrxInverse << m00, m01, m02, m10, m11, m12 , m20, m21, m22;
    
    return (jacobianMatrx.transpose() * jacobianMatrxInverse.inverse());

}



void IK::calculateAngles(VectorXf dxdydz, float angle) {
    
    VectorXf dAngles = VectorXf::Zero(mBodys.size() * 2, 1);
    
    
    
    dAngles = jacobianInverse(angle) * dxdydz;
    
    
    
  //  std::cout <<"jacobianInverse():" <<  jacobianInverse(angle) << "\n";
  //  std::cout <<"dAngles:" <<  dAngles << "\n";
    
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        for (int j = 0; j < 2; j++) {
        
            mBodys[i]->mAngles[j] += dAngles(i*2 + j);
            
        }
    }
    
   
    for (int i = 0; i < mBodys.size(); i++) {
        
        for (int j = 0; j < 2; j++) {
        
            while (mBodys[i]->mAngles[j] < -1 * PI) {
                
                mBodys[i]->mAngles[j] += 2 * PI;
            }
            
            while (mBodys[i]->mAngles[j] >  PI) {
                
                mBodys[i]->mAngles[j] -= 2 * PI;
            }
            
            
        }
    }
    
    
    updateCurPosition(angle, mArmStartPostion);
     
    
    
}

bool IK::getGoal() {
    
 //   std::cout <<"fabs(mCurPosition(0) - mGoalPostion(0)):" << fabs(mCurPosition(0) - mGoalPostion(0)) << "\n";
    
 //   std::cout <<"fabs(mCurPosition(1) - mGoalPostion(1)):" << fabs(mCurPosition(1) - mGoalPostion(1)) << "\n";
    
    
 //   std::cout <<"mCurPosition:" << mCurPosition(0) << "," << mCurPosition(1) << "," << mCurPosition(2) << "\n";
    
    
    if (fabs(mCurPosition(0) - mGoalPostion(0)) <= 0.1f && fabs(mCurPosition(1) - mGoalPostion(1)) <= 0.1f && fabs(mCurPosition(2) - mGoalPostion(2)) <= 0.1f) {
        
        std::cout << "get goal true \n";
        return true;
    }
    
    std::cout << "get goal false \n";
    return false;
}


float IK::rotateBody() {
    
    Vector3f directionGoal;
    directionGoal(0) = mGoalPostion(0) - mOffset;
    directionGoal(1) = 0;
    directionGoal(2) = mGoalPostion(2);
    
 //    std::cout << "directionGoal: " << directionGoal <<"\n";
    
    float cosAngle = directionGoal.dot(headDirction);   // ignore |mGoalPosition| and |mCurposition| since we only want the sign of cosAngle
        
    cosAngle = cosAngle * 1.0 / (sqrt(pow(directionGoal(0),2) + pow(directionGoal(1),2) + pow(directionGoal(2),2))  * sqrt(pow(headDirction(0),2) + pow(headDirction(1),2) + pow(headDirction(2),2)));
    
        
    float angle = acos(cosAngle);
        
    Vector3f directionNew;
        
    directionNew(0) = - 1 * sin(angle);
    directionNew(1) = headDirction(1);
    directionNew(2) = - 1 * cos(angle);
    
    
    
  //  std::cout << "directionNew1: " << directionNew <<"\n";
    
   
    
    float cosAngleNew = directionGoal.dot(directionNew);
    
    cosAngleNew = cosAngleNew * 1.0 / (sqrt(pow(directionGoal(0),2) + pow(directionGoal(1),2) + pow(directionGoal(2),2))  * sqrt(pow(directionNew(0),2) + pow(directionNew(1),2) + pow(directionNew(2),2)));
    
  //   std::cout << "cosAngleNew: " << cosAngleNew << " \n";
    if (cosAngleNew >0.9  && cosAngleNew < 1.1) {
        
        // update direction
        
        std::cout << "need to rotate body 1 \n";
        std::cout << "Angle: " << angle << "\n";
        
        return angle;
    }
    
    angle *= -1;
    
    std::cout << "need to rotate body 2 \n";
    std::cout << "Angle: " << angle << "\n";
    
    return angle;
}



void IK::updateHeadDir(float angle) {
    
    //update head direction
    headDirction(0) = - 1 * sin(angle);
    headDirction(2) = - 1 * cos(angle);
    
    
  //  std::cout << "headDirction: " << headDirction << "\n";
}

void IK::updateArmPosition(Vector3f headDirction_var, Vector3f mPelPostion_var) {
    
    // update arm start position
    
    Vector2f newPelDir;
    newPelDir(0) = headDirction_var(0);
    newPelDir(1) = headDirction_var(2);
    
    Matrix2f rotateMatrix;
    rotateMatrix << 0.0f, -1.0f, 1.0f, 0.0f;
    
    Vector2f newArmDir = rotateMatrix * newPelDir;
    
 //   std::cout <<"new newArmDir:" << newArmDir(0) << "," << newArmDir(1) << "\n";
    
    mArmStartPostion(0) = newArmDir(0) * 0.6 + mPelPostion_var(0);
    mArmStartPostion(1) = mPelPostion_var(1) + 1.2;
    mArmStartPostion(2) = newArmDir(1) * 0.6 + mPelPostion_var(2);
    
  //  std::cout <<"new mArmStartPostion:" << mArmStartPostion(0) << "," << mArmStartPostion(1) << "," << mArmStartPostion(2) << "\n";
    
}


void IK :: updateCurPosition(float angle, Vector3f mArmStartPostion_var) {
    
    float angle_z = 0;
    float angle_y = 0;
    
    mCurPosition(0) = mArmStartPostion_var(0);
    mCurPosition(1) = mArmStartPostion_var(1);
    mCurPosition(2) = mArmStartPostion_var(2);
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        angle_z += mBodys[i]->mAngles[0];
        angle_y += mBodys[i]->mAngles[1];
        
      //  mCurPosition(0) += mBodys[i]->mLen * sin(angle_z) * cos(angle_y);
      //  mCurPosition(1) -= mBodys[i]->mLen * cos(angle_z);
      //  mCurPosition(2) -= mBodys[i]->mLen * sin(angle_z) * sin(angle_y);
        
        
        mCurPosition(0) += (cos(angle) * mBodys[i]->mLen * sin(angle_z) * cos(angle_y) + sin(angle) * mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
        mCurPosition(1) -= mBodys[i]->mLen * cos(angle_z);
        mCurPosition(2) += (sin(angle) * mBodys[i]->mLen * sin(angle_z) * cos(angle_y) - cos(angle) * mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
    }
  
    
    std::cout << "new mCurPosition: " << mCurPosition << "\n";
    
}

void IK :: updateCurPositionInitial( Vector3f mArmStartPostion_var) {
    
    mCurPosition(0) = mArmStartPostion_var(0);
    mCurPosition(1) = mArmStartPostion_var(1) - 1.5 - 0.18;
    mCurPosition(2) = mArmStartPostion_var(2);
    
  //  std::cout << "new intial mCurPosition: " << mCurPosition << "\n";
    
}


float IK::dp (float p1, float p2) {
    
    
    
    if (p1 - p2 == 0) {
        
        return 0.0;
        
    } else if (p1 - p2  > 0.5) {
        
        return 0.1;
        
    } else if (p1 - p2 > 0){
        
        return 0.05;
    } else if (p1 - p2 > -0.5) {
        
        return -0.05;
    } else {
        
        return -0.1;
    }
    
}

float IK::walk(ModelerApplication* app) {
    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(1);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(7);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(4);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(8);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(43);
   
    
    float dt = 0.5;
    
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(7, dt, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(8, dt, 0);
    app->GetUI()->m_pwndGraphWidget->AddCtrlPt(43,dt, 0);
    
    
    updateFootPosition(0, "l");
    updateFootPosition(0, "r");
    
    
    std::cout <<  mFootPosition << ": mFootPosition. \n";
    
    bool first = true;
    
    
    VectorXf angles = VectorXf :: Zero(8);
    
    while (dt < 8) {
        
        if (first) {
        
            dt += 0.5;
            
            // left forward
            std::cout <<"dt: "<< dt <<"\n";
            
            VectorXf dydz = VectorXf::Zero(2, 1);
            
            dydz(0) = 0.457836 - mFootPosition(1) - 0.15;
            dydz(1) = -1.15 - mFootPosition(2);
            
            walk_calculateAngles(dydz, 0, "l");
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, mLegs[0]->mAngle * 180.0 / PI);
            angles(0) = mLegs[0]->mAngle * 180.0 / PI;
            
            cout << "need angle:" <<angles(0) << endl;

            
            dt += 1;
            
            // right keep
            
            float dp_y = mFootPosition(1) - 0.15;
            float dp_z = mFootPosition(2) - 0;
            
            dydz(0) = -(mFootPosition(1) - 0.15) ;
            dydz(1) = -(mFootPosition(2) - 0);
            
            walk_calculateAngles(dydz, 0, "r");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, mLegs[1]->mAngle * 180.0 / PI);
            
            angles(1) = mLegs[1]->mAngle * 180.0 / PI;

            
            
            // pelv forward
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(43, dt, mFootPosition(2) + mPelPostion(2));
            float temp = mFootPosition(2);
            
            
            // left foot back
            
            walk_calculateAngles(dydz, 0, "l");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, mLegs[0]->mAngle * 180.0 / PI);
            angles(2) = mLegs[0]->mAngle * 180.0 / PI;
            
        
            dt += 0.5;
            
            // right forward
            dydz(0) = dp_y;
            dydz(1) = dp_z;
            
            walk_calculateAngles(dydz, 0, "r");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, mLegs[1]->mAngle * 180.0 / PI);
            angles(3) = mLegs[1]->mAngle * 180.0 / PI;
        
            mPelPostion(2) += temp;
            
            
            
            if (isNearGoal(mPelPostion, headDirction)) break;
            
            
            dt += 0.5;
            
            // right forward
            dydz(0) = 0.457836 - mRFootPosition(1);
            dydz(1) = -1.15 - mRFootPosition(2);
            
            walk_calculateAngles(dydz, 0, "r");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, mLegs[1]->mAngle * 180.0 / PI);
            angles(4) = mLegs[1]->mAngle * 180.0 / PI;
            
            //std::cout << "finish right forward 2. \n";
            
            
            
            dt += 1;
            
            // left keep
            dp_y = mRFootPosition(1) - 0.15;
            dp_z = mRFootPosition(2);
            
            dydz(0) = -(mRFootPosition(1) - 0.15);
            dydz(1) = -(mRFootPosition(2));
            
            walk_calculateAngles(dydz, 0, "l");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, mLegs[0]->mAngle * 180.0 / PI);
            angles(5) = mLegs[0]->mAngle * 180.0 / PI;
            
            //    std::cout << "finish left keep 2. \n";
            
            
            // pel forward
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(43, dt, mPelPostion(2) + mRFootPosition(2));
            temp = mRFootPosition(2);
            
            // right back
            
            walk_calculateAngles(dydz, 0, "r");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, mLegs[1]->mAngle * 180.0 / PI);
            angles(6) = mLegs[1]->mAngle * 180.0 / PI;
            //   std::cout << "finish right back 2. \n";
            
            
            
            
            dt += 0.5;
            // left forward
            
            //    std::cout << "left forward dt: " << dt <<"\n";
            
            dydz(0) = dp_y;
            dydz(1) = dp_z;
            
            //      std::cout << "dydz(1): " << dydz(1) <<"\n";
            
            walk_calculateAngles(dydz, 0, "l");
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, mLegs[0]->mAngle * 180.0 / PI - 90);
            angles(7) = mLegs[0]->mAngle * 180.0 / PI - 90;
            //    std::cout << "finish left forward 2. \n";
            
            mPelPostion(2) += temp;
            
            //    std::cout << "mple "<< mPelPostion(2) <<"\n";
            
            if (isNearGoal(mPelPostion, headDirction)) break;
            
            first = false;
            
        } else {
            
            
            dt += 0.5;
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, angles(0));
            mLegs[0]->mAngle = angles(0) * PI / 180;
            updateFootPosition(0, "l");
            
            dt += 1;
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, angles(1));
            mLegs[1]->mAngle = angles(1) * PI / 180;
            updateFootPosition(0, "r");
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(43, dt, mFootPosition(2) + mPelPostion(2));
            float temp = mFootPosition(2);
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, angles(2) );
            mLegs[0]->mAngle = angles(2) * PI / 180;
            updateFootPosition(0, "l");
            
            
            dt += 0.5;
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, angles(3));
            mLegs[1]->mAngle = angles(3) * PI / 180;
            updateFootPosition(0, "r");
            
            mPelPostion(2) += temp;
            
            if (isNearGoal(mPelPostion, headDirction)) break;
            
            dt += 0.5;
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, angles(4));
            mLegs[1]->mAngle  = angles(4) * PI / 180;
            updateFootPosition(0, "r");
            
            
            dt += 1;
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, angles(5));
            mLegs[0]->mAngle  = angles(5) * PI / 180;
            updateFootPosition(0, "l");
            
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(43, dt, mPelPostion(2) + mRFootPosition(2));
            temp = mRFootPosition(2);
            
            
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(4, dt, angles(6));
            mLegs[1]->mAngle = angles(6) * PI / 180;
            updateFootPosition(0, "r");
            
            
            dt += 0.5;
            app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, dt, angles(7));
            mLegs[0]->mAngle = angles(7) * PI / 180;
            updateFootPosition(0, "l");
            
            mPelPostion(2) += temp;
            
            if (isNearGoal(mPelPostion, headDirction)) break;
        }
        
    }
    
    cout << "dt: " << dt << endl;
    
    return dt;
    
}


MatrixXf IK::walk_jacobianInverse(float angle, std::string lr) {
    
    
    MatrixXf walk_jacobianMatrx = MatrixXf::Zero(2, mLegs.size());
    
    int start = 0;
    int end = 0;
    
    if (lr.compare("l") == 0) {
        start = 0;
        end = 1;
    } else {
        
        start = 1;
        end = 2;
    }
    
    
    for (int j = start; j < end; j++) {
        
        float angle_x = 0;
        
         for (int i = start; i < end; i++) {
            
            angle_x += mLegs[i]->mAngle;
           
            if (i >= j) {
                walk_jacobianMatrx(0, j - start) += (float)(mLegs[i]->mLen * sin(angle_x));
                walk_jacobianMatrx(1, j - start) += (float)(-mLegs[i]->mLen * cos(angle_x));
            }
        }
    }
    
    MatrixXf jacobianMatrxInverse1 = MatrixXf::Zero(2,2);
    jacobianMatrxInverse1 = walk_jacobianMatrx * (walk_jacobianMatrx.transpose());
    
    
    float m00 = roundf(jacobianMatrxInverse1(0,0) * 1000000) / 1000000.0;
    float m01 = roundf(jacobianMatrxInverse1(0,1) * 1000000) / 1000000.0;
    float m10 = roundf(jacobianMatrxInverse1(1,0) * 1000000) / 1000000.0;
    float m11 = roundf(jacobianMatrxInverse1(1,1) * 1000000) / 1000000.0;
    
    
    Matrix2f walk_jacobianMatrxInverse;
    walk_jacobianMatrxInverse << m00, m01, m10, m11;
    return walk_jacobianMatrx.transpose() * walk_jacobianMatrxInverse.inverse();
}




void IK::walk_calculateAngles(VectorXf dydz, float angle, std::string lr) {
    
    VectorXf walk_dAngles = VectorXf::Zero(mLegs.size() - 1, 1);
    walk_dAngles = walk_jacobianInverse(angle, lr) * dydz;
    
    
    int start = 0;
    int end = 0;
    
    if (lr.compare("l") == 0) {
        start = 0;
        end = 1;
    } else {
        
        start = 1;
        end = 2;
    }

    
    for (int i = start; i < end; i++) {
            
            mLegs[i]->mAngle += walk_dAngles(i - start);
    }
    
    
    for (int i = start; i < end; i++) {
            
            while (mLegs[i]->mAngle < -1 * PI) {
                
                mLegs[i]->mAngle += (2 * PI);
            }
            
            while (mLegs[i]->mAngle > PI) {
                
                mLegs[i]->mAngle -= (2 * PI);
            }
    }
    
    updateFootPosition(angle, lr);
}
    
    
void IK :: updateFootPosition(float angle, std::string lr) {
    
    int start = 0;
    int end = 0;
    
    if (lr.compare("l") == 0) {
        start = 0;
        end = 1;
    } else {
        
        start = 1;
        end = 2;
        
    }
    
    float angle_x = 0;
    
    if (lr.compare("l") == 0) {
    
        mFootPosition(0) = mPelPostion(0) + 0.3;
        mFootPosition(1) = mPelPostion(1);
        mFootPosition(2) = 0;
        
        for (int i = start; i < end; i++) {
            
            angle_x += mLegs[i]->mAngle;
            
            mFootPosition(1) -= mLegs[i]->mLen * cos(angle_x);
            mFootPosition(2) -= mLegs[i]->mLen * sin(angle_x);
        }
        
     //    std::cout << "new mFootPosition: " << lr << ": "<< mFootPosition << "\n";
        
    } else {
        
         mRFootPosition(0) = mPelPostion(0) - 0.3;
         mRFootPosition(1) = mPelPostion(1);
         mRFootPosition(2) = 0;
        
        for (int i = start; i < end; i++) {
            
            angle_x += mLegs[i]->mAngle;
            
            mRFootPosition(1) -= mLegs[i]->mLen * cos(angle_x);
            mRFootPosition(2) -= mLegs[i]->mLen * sin(angle_x);
        }
        
      //   std::cout << "new mRFootPosition: " << lr << ": "<< mRFootPosition << "\n";
    }
}


bool IK::isNearGoal(Vector3f mPelPostion_Var, Vector3f headDirction_var) {
    
    
    std::cout << "mPelPostion_Var: " << mPelPostion_Var << "\n";
   // std::cout << "headDirction_var: " << headDirction_var << "\n";
    
    mPelPostion_Var(0) = mOffset + fabs(mPelPostion_Var(2)) * headDirction_var(0);
    mPelPostion_Var(2) = fabs(mPelPostion_Var(2)) * headDirction_var(2);
    
    
    std::cout << "mPelPostion_Var: " << mPelPostion_Var << "\n";
    
    updateArmPosition(headDirction_var, mPelPostion_Var);
    
    
    std::cout << "mArmStartPostion: " << mArmStartPostion<< "\n";
    
    float distance = sqrt(pow(mGoalPostion(0) - mArmStartPostion(0), 2) + pow(mGoalPostion(1) - mArmStartPostion(1), 2) + pow(mGoalPostion(2) - mArmStartPostion(2), 2));
    
    
    if ( distance > 1.78) {
        
        cout << "long distance" << distance<<" \n";
        
        return false;
    }
    
    
     cout << "short distance" << distance<<" \n";
    return true;
    
}



