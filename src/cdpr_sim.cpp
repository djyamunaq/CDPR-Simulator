#include <eigen3/Eigen/Dense>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include<cmath>

#define WHITE cv::Scalar(255, 255, 255)
#define BLACK cv::Scalar(0, 0, 0)
#define GREEN cv::Scalar(255, 255, 0)
#define BLUE cv::Scalar(255, 0, 0)
#define RED cv::Scalar(0, 0, 255)

using std::cout;
using std::endl;

cv::Mat img;

int main() {
    /* Window size */
    const int img_h = 400;
    const int img_w = 600;

    /* Joint variables -> motor positions and variations */
    double j1, j2, j3, j4, dj1, dj2, dj3, dj4;
    j1 = j2 = j3 = j4 = 0;
    dj1 = dj2 = dj3 = dj4 = 0;

    /* Transformation matrix from end-effector frame to world frame */
    Eigen::Matrix3d T;
    /* End-effector size */
    const int ee_l = 30;
    /* End-effector corners positions in end-effector frame */
    Eigen::Matrix<double, 3, 4> X_ee_fr;
    X_ee_fr <<  -ee_l/2, -ee_l/2, ee_l/2, ee_l/2,
                -ee_l/2, ee_l/2, ee_l/2, -ee_l/2,
                1, 1, 1, 1;
    /* End-effector corners positions in world frame */
    Eigen::Matrix<double, 3, 4> X;
    /* End-effector frame position in world frame */
    Eigen::Vector2d Oe_X;
    /* End-effector orientation angle in world frame */
    double theta;

    /* End effector frame initial and final position in world frame */
    Eigen::Vector2d Oe_Xi;
    Eigen::Vector2d Oe_Xf;
    /* End effector initial and final orientation in world frame */
    double theta_i;
    double theta_f;
    
    /* Initial state */
    Oe_Xi << img_w/2, img_h/2;
    theta_i = 0;
    /* Final State */
    // Oe_Xf << (img_w)/2, (img_h)/2;
    // Oe_Xf << (img_w+ee_l)/2, (img_h-ee_l)/2;
    Oe_Xf << 60, 60;
    theta_f = 45;
    theta_f *= (M_PI/180);

    /* Time interval for simulation */
    const double Dt = 1;
    /* Time interval between frames */
    const double dt = 1e-3;
    /* Variations in x and y position and theta orientation for each time frame */
    const double dx = dt*(Oe_Xf(0) - Oe_Xi(0))/Dt;
    const double dy = dt*(Oe_Xf(1) - Oe_Xi(1))/Dt;
    const double dth = dt*(theta_f - theta_i)/Dt;

    Oe_X = Oe_Xi;
    theta = theta_i;
    
    /* Calculate trajectory */
    for(double t=0; t<Dt; t+=dt) {

        /* Pulley configuration */
            /* Pulleys radius */
        double pulley_r = 10;
            /* Pulleys Up Left */
        Eigen::Vector2d pulley_1_c = {pulley_r, pulley_r};
            /* Pulleys Down Left */
        Eigen::Vector2d pulley_2_c = {pulley_r, img_h-pulley_r}; 
            /* Pulleys Down Right */
        Eigen::Vector2d pulley_3_c = {img_w-pulley_r, img_h-pulley_r}; 
            /* Pulleys Up Right */
        Eigen::Vector2d pulley_4_c = {img_w-pulley_r, pulley_r}; 

        /* End-effector configuration */
            /* End-effector thickness */
        double ee_t = 5;
            /* Get previous Transformation Matrix */
        T <<    cos(theta), -sin(theta), Oe_X(0),
                sin(theta), cos(theta), Oe_X(1),
                0, 0, 1;
            /* End effector previous position in world frame */
        Eigen::Matrix<double, 3, 4> X_prev = T*X_ee_fr;
            /* Update position and orientation of end-effector in world frame */
        Oe_X(0) += dx;
        Oe_X(1) += dy;
        theta += dth;
            /* Update Transformation Matrix */
        T <<    cos(theta), -sin(theta), Oe_X(0),
                sin(theta), cos(theta), Oe_X(1),
                0, 0, 1;
            /* End effector position in world frame */
        X = T*X_ee_fr;

        /* Wires config */
            /* Wire diameter */
        double wire_d = 1;
            /* Wire 1 -> pulley 1 x point 4 ee */
        Eigen::Vector2d wire_1_start = {(pulley_1_c(0) + pulley_r), (pulley_1_c(1) - pulley_r)};
        Eigen::Vector2d wire_1_end = {X(0, 3), X(1, 3)};        
            /* Wire 2 -> pulley 2 x point 3 ee */
        Eigen::Vector2d wire_2_start = {(pulley_2_c(0) + pulley_r), (pulley_2_c(1) + pulley_r)};
        Eigen::Vector2d wire_2_end = {X(0, 2), X(1, 2)};
            /* Wire 3 -> pulley 3 x point 2 ee */
        Eigen::Vector2d wire_3_start = {(pulley_3_c(0) - pulley_r), (pulley_3_c(1) + pulley_r)};
        Eigen::Vector2d wire_3_end = {X(0, 1), X(1, 1)};
            /* Wire 4 -> pulley 4 x point 1 ee */
        Eigen::Vector2d wire_4_start = {(pulley_4_c(0) - pulley_r), (pulley_4_c(1) - pulley_r)};
        Eigen::Vector2d wire_4_end = {X(0, 0), X(1, 0)};

        /* ============================================================================================= */
        /* Inverse Kinematics */
            /* Wire i size: li = norm2(pi - ej) where pi is the vector of the pulley i in the world frame and ej is the vector of end-effector corner connected to pi also in the world frame */
        
            /* Get previous wires positions */
        Eigen::Vector2d w1_v_prev = {pulley_1_c(0) - X_prev(0, 0), pulley_1_c(1) - X_prev(1, 0)}; 
        Eigen::Vector2d w2_v_prev = {pulley_2_c(0) - X_prev(0, 1), pulley_2_c(1) - X_prev(1, 1)}; 
        Eigen::Vector2d w3_v_prev = {pulley_3_c(0) - X_prev(0, 2), pulley_3_c(1) - X_prev(1, 2)}; 
        Eigen::Vector2d w4_v_prev = {pulley_4_c(0) - X_prev(0, 3), pulley_4_c(1) - X_prev(1, 3)};
            /* Update wires positions */
        Eigen::Vector2d w1_v = {pulley_1_c(0) - X(0, 0), pulley_1_c(1) - X(1, 0)}; 
        Eigen::Vector2d w2_v = {pulley_2_c(0) - X(0, 1), pulley_2_c(1) - X(1, 1)}; 
        Eigen::Vector2d w3_v = {pulley_3_c(0) - X(0, 2), pulley_3_c(1) - X(1, 2)}; 
        Eigen::Vector2d w4_v = {pulley_4_c(0) - X(0, 3), pulley_4_c(1) - X(1, 3)};

        double l1 = w1_v.norm(); 
        double l2 = w2_v.norm(); 
        double l3 = w3_v.norm(); 
        double l4 = w4_v.norm();

        double l1_prev = w1_v_prev.norm(); 
        double l2_prev = w2_v_prev.norm(); 
        double l3_prev = w3_v_prev.norm(); 
        double l4_prev = w4_v_prev.norm();

            /* Motor radius */
        double mr = 5; 
            /* Calculate position in motor */
        double dl_1 = l1 - l1_prev;
        double dl_2 = l2 - l2_prev;
        double dl_3 = l3 - l3_prev;
        double dl_4 = l4 - l4_prev;

        dj1 = dl_1/(2*M_PI*mr);
        dj2 = -dl_2/(2*M_PI*mr);
        dj3 = dl_3/(2*M_PI*mr);
        dj4 = -dl_4/(2*M_PI*mr);

        j1 += dj1;
        j2 += dj2;
        j3 += dj3;
        j4 += dj4;

        /* Print state */
        cout << "==========================" << endl;
        // cout << "> Position:\n" << X.block<2, 4>(0, 0).transpose() << endl;
        cout << "> Position:\n" << X.block<2, 1>(0, 1).transpose() << endl;
        // cout << "> Position eefr:\n" << X_ee_fr.block<2, 4>(0, 0).transpose() << endl;
        cout << "> Orientation:\t" << 180*theta/M_PI << endl;
        cout << "Wires sizes:" << endl;
        cout << "\tl1: " << l1 << endl; 
        cout << "\tl2: " << l2 << endl; 
        cout << "\tl3: " << l3 << endl; 
        cout << "\tl4: " << l4 << endl; 
        cout << "Motors positions:" << endl;
        cout << "\tj1: " << j1 << endl; 
        cout << "\tj2: " << j2 << endl; 
        cout << "\tj3: " << j3 << endl; 
        cout << "\tj4: " << j4 << endl; 
        // cout << "> Transformation Matrix: " << endl;
        // cout << T << endl;

        /* ============================================================================================= */
        /* Draw scenario */

        /* Reset background */
        img.create(400, 600, CV_8UC3);
        img.setTo(cv::Scalar(0, 0, 0));
        cv::rectangle(img, cv::Point(0, 0), cv::Point(img_w, img_h), cv::Scalar(255, 255, 255), 10, cv::LINE_8, 0);

        /* Update figures */
            /* End-effector */
        cv::line(img, cv::Point(X(0, 0), X(1, 0)), cv::Point(X(0, 1), X(1, 1)), WHITE, ee_t);
        cv::line(img, cv::Point(X(0, 1), X(1, 1)), cv::Point(X(0, 2), X(1, 2)), WHITE, ee_t);
        cv::line(img, cv::Point(X(0, 2), X(1, 2)), cv::Point(X(0, 3), X(1, 3)), WHITE, ee_t);
        cv::line(img, cv::Point(X(0, 3), X(1, 3)), cv::Point(X(0, 0), X(1, 0)), WHITE, ee_t);
            /* Pulleys */
        cv::circle(img, cv::Point(pulley_1_c(0), pulley_1_c(1)), pulley_r, GREEN, 10);
        cv::circle(img, cv::Point(pulley_2_c(0), pulley_2_c(1)), pulley_r, GREEN, 10);
        cv::circle(img, cv::Point(pulley_3_c(0), pulley_3_c(1)), pulley_r, GREEN, 10);
        cv::circle(img, cv::Point(pulley_4_c(0), pulley_4_c(1)), pulley_r, GREEN, 10);
            /* Wires */
        cv::line(img, cv::Point(wire_1_start(0), wire_1_start(1)), cv::Point(wire_1_end(0), wire_1_end(1)), RED, wire_d);
        cv::line(img, cv::Point(wire_2_start(0), wire_2_start(1)), cv::Point(wire_2_end(0), wire_2_end(1)), RED, wire_d);
        cv::line(img, cv::Point(wire_3_start(0), wire_3_start(1)), cv::Point(wire_3_end(0), wire_3_end(1)), RED, wire_d);
        cv::line(img, cv::Point(wire_4_start(0), wire_4_start(1)), cv::Point(wire_4_end(0), wire_4_end(1)), RED, wire_d);
            /* Motors */
        cv::line(img, cv::Point(pulley_1_c(0), pulley_1_c(1)), cv::Point(pulley_1_c(0) + (pulley_r+5)*cos(j1*2*M_PI), pulley_1_c(1) + (pulley_r+5)*sin(j1*2*M_PI)), BLUE, 3);
        cv::line(img, cv::Point(pulley_2_c(0), pulley_2_c(1)), cv::Point(pulley_2_c(0) + (pulley_r+5)*cos(j2*2*M_PI), pulley_2_c(1) + (pulley_r+5)*sin(j2*2*M_PI)), BLUE, 3);
        cv::line(img, cv::Point(pulley_3_c(0), pulley_3_c(1)), cv::Point(pulley_3_c(0) + (pulley_r+5)*cos(j3*2*M_PI), pulley_3_c(1) + (pulley_r+5)*sin(j3*2*M_PI)), BLUE, 3);
        cv::line(img, cv::Point(pulley_4_c(0), pulley_4_c(1)), cv::Point(pulley_4_c(0) + (pulley_r+5)*cos(j4*2*M_PI), pulley_4_c(1) + (pulley_r+5)*sin(j4*2*M_PI)), BLUE, 3);

        cv::namedWindow("map", cv::WINDOW_AUTOSIZE); 
        cv::moveWindow("map", 1020, 0);              
        cv::imshow("map", img);  

        /* Delay 1 ms */
        cv::waitKey(10);
    }

    return 0;
}