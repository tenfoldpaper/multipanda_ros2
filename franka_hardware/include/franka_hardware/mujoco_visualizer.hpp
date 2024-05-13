#pragma once
#include "mujoco/mujoco.h"

#include <iostream>
#include <map>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <string>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <exception>
#include <GLFW/glfw3.h>

namespace franka_hardware{
class MujocoVisualizer{
public:
    MujocoVisualizer() = delete;
    MujocoVisualizer(MujocoVisualizer const&);
    ~MujocoVisualizer();
    MujocoVisualizer(mjModel* &m, mjData* &d) : m_(m), d_(d){};

    /* Mujoco setup functions */
    void startMujoco(){
        // // init GLFW
        if( !glfwInit() )
            mju_error("Could not initialize GLFW");

        // // create window, make OpenGL context current, request v-sync
        GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_makeScene(m_, &scn_, 2000);                // space for 2000 objects
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);   // model-specific context

        // install GLFW mouse and keyboard callbacks
        // Solution from BlueParalel at https://old.reddit.com/r/opengl/comments/15khvm0/need_help_with_callback_functions/
        glfwSetWindowUserPointer(window, this);
        glfwSetCursorPosCallback(window, [](GLFWwindow* w, double xpos, double ypos)
        {
            auto mjv = reinterpret_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(w));
            // no buttons down: nothing to do
            if( !mjv->button_left_ && !mjv->button_middle_ && !mjv->button_right_ )
                return;

            // compute mouse displacement, save
            double dx = xpos - mjv->lastx_;
            double dy = ypos - mjv->lasty_;
            mjv->lastx_ = xpos;
            mjv->lasty_ = ypos;

            // get current window size
            int width, height;
            glfwGetWindowSize(w, &width, &height);

            // get shift key state
            bool mod_shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                            glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

            // determine action based on mouse button
            mjtMouse action;
            if( mjv->button_right_ )
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            else if( mjv->button_left_ )
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            else
                action = mjMOUSE_ZOOM;

            // move camera
            mjv_moveCamera(mjv->m_, action, dx/height, dy/height, &mjv->scn_, &mjv->cam_);
        });
        glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int button, int act, int mods)
        {
            auto mjv = reinterpret_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(w));
            // update button state
            mjv->button_left_ =   (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
            mjv->button_middle_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
            mjv->button_right_ =  (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

            // update mouse position
            glfwGetCursorPos(w, &mjv->lastx_, &mjv->lasty_);
        });
        glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset)
        {
            auto mjv = reinterpret_cast<MujocoVisualizer*>(glfwGetWindowUserPointer(window));
            // emulate vertical mouse motion = 5% of window height
            mjv_moveCamera(mjv->m_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mjv->scn_, &mjv->cam_);
        });
        // adding a keyboard callback wasd for moving the camera around would be nice
        // glfwSetKeyCallback(window, keyboard);

        double arr_view[] = {90, -5, 5, 0.012768, -0.000000, 1.254336};
        cam_.azimuth = arr_view[0];
        cam_.elevation = arr_view[1];
        cam_.distance = arr_view[2];
        cam_.lookat[0] = arr_view[3];
        cam_.lookat[1] = arr_view[4];
        // std::cout << !glfwWindowShouldClose(window_) << std::endl;
        while( !glfwWindowShouldClose(window))
        {
        // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // update scene and render
            // mjtNum simstart = d_->time;
            // while( d_->time - simstart < 1.0/30.0 )
            // {
            //     boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
            // }
            // std::cout << d_->time - simstart << std::endl;

            {
                mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
                mjr_render(viewport, &scn_, &con_);
            }
            //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
            
            boost::this_thread::sleep_for(boost::chrono::milliseconds(33)); // 30 fps

        }
        // free visualization storage
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);

        // free MuJoCo model and data, deactivate
        // mj_deleteData(d);
        // mj_deleteModel(m);
    }

private:
    /* Mujoco variables */
    mjModel* m_;
    mjData* d_;
    mjvCamera cam_;                      // abstract camera
    mjvOption opt_;                      // visualization options
    mjvScene scn_;                       // abstract scene
    mjrContext con_;                     // custom GPU context
    char error[1000] = {""};

    /* GLFW window-related stuff */
    bool button_left_;
    bool button_middle_;
    bool button_right_;

    double lastx_ = 0;
    double lasty_ = 0;
    // keyboard callback
    // void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
    // {
    //     // backspace: reset simulation
    //     if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    //     {
    //         mj_resetData(m_, d_);
    //         mj_forward(m_, d_);
    //     }
    // }
};
} // namespace franka_hardware