#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h> 
#include <pybind11/stl.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "_env.hpp"

namespace py = pybind11;

cv::Mat matM (double nu, double omega, double time, double *stds){
    double tmp[2][2] = {{std::pow(stds[0],2)*std::abs(nu)/time + std::pow(stds[1],2)*std::abs(omega)/time, 0},
                        {0, std::pow(stds[2],2)*std::abs(nu)/time + std::pow(stds[3],2)*std::abs(omega)/time}};
    cv::Mat M = cv::Mat(2, 2, CV_64F, tmp);
    return M;
}

cv::Mat matA(double nu, double omega, double time, double theta){
    double st = std::sin(theta);
    double ct = std::cos(theta);
    double stw = std::sin(theta + omega*time);
    double ctw = std::cos(theta + omega*time);
    double tmp[3][2] = {{(stw-st)/omega, -nu/std::pow(omega,2)*(stw-st) + nu/omega*time*ctw},
                        {(-ctw+ct)/omega, -nu/std::pow(omega,2)*(-ctw+ct) + nu/omega*time*stw},
                        {0, time}};
    cv::Mat A = cv::Mat(3, 2, CV_64F, tmp);
    return A;
}

cv::Mat matF(double nu, double omega, double time, double theta){
    cv::Mat F = cv::Mat::eye(3, 3, CV_64F);
    F.at<double>(0, 2) = nu/omega * (std::cos(theta + omega*time) - std::cos(theta));
    F.at<double>(1, 2) = nu/omega * (std::sin(theta + omega*time) - std::sin(theta));
    return F;
}

cv::Mat matH(double *pose, double *landmark_pos){
    double mx = landmark_pos[0];
    double my = landmark_pos[1];
    double mux = pose[0];
    double muy = pose[1];
    double q = std::pow((mux - mx),2) + std::pow((muy - my),2);
    double tmp[2][3] = {{(mux-mx)/std::sqrt(q), (muy-my)/std::sqrt(1), 0.0},
                        {(mux-mx)/q, (mux-my)/1, -1.0}};
    cv::Mat H = cv::Mat(2, 3, CV_64F, tmp);
    return H;
}

cv::Mat matQ(double distance_dev, double direction_dev){
    double tmp[2][2] = {{std::pow(distance_dev,2), 0},
                        {0, std::pow(direction_dev,2)}};
    cv::Mat Q = cv::Mat(2, 2, CV_64F, tmp);
    return Q;
}
        
class KalmanFilter {
public:
    KalmanFilter(Map & map, 
                 py::array<double> & init_pos, 
                 py::dict & motion_noise_std, 
                 double distance_div_rate, 
                 double direction_dev):
    distance_dev_rate(distance_dev_rate),
    direction_dev(direction_dev){
        m_map = map;
        py::buffer_info buf = init_pos.request();
        cout<<buf.shape[0]<<" "<<buf.shape[1]<<endl;
        m_belief_mean = cv::Mat(buf.shape[0], buf.shape[1], CV_64_F, buf.ptr);
        m_belief_cov =  cv::Mat::eye(3, 3, CV_64_F)*0.1;
        m_pose = m_belief_mean;
        m_motions_noise_stds[0] = motion_noise_std["nn"];
        m_motions_noise_stds[1] = motion_noise_std["no"];
        m_motions_noise_stds[2] = motion_noise_std["on"];
        m_motions_noise_stds[3] = motion_noise_std["oo"];
    }
    /*
    void observation_update(const std::vector<std::pair<py::array<double> & pos, std::size_t id>>> & observation){
        for(size_t i=0; i<observation.size(); ++i){
            
        }
    }
        
    void motion_update(nu, omega, time){
        if (std::abs(omega)< 1e-5){ omega = 1e-5; }
        cv::Mat M = matM(nu, omega, time, m_motion_noise_stds);
        cv::Mat A = matA(nu, omega, time, m_belief_mean[2]);
        cv::Mat F = matF(nu, omega, time, m_belief_mean[2]);
        m_belief_cov = F.dot(m_belief_cov).dot(F.T())+A.dot(M).dot(A.T());
        if (std::fabs(omega) < 1e-10){
            m_belief_mean =  m_belief_mean + 
                np.array([nu*math.cos(t0),
                                     nu*math.sin(t0),
                                     omega])*time
        else:
            m_belief_mean =  m_belief_mean + np.array([nu/omega*(math.sin(t0 + omega*time) - math.sin(t0)),
                                     nu/omega*(-math.cos(t0 + omega*time) + math.cos(t0)),
                                     omega*time])
       
    }
    */
            
public:
    Map m_map;
    cv::Mat pose;
    cv::Mat m_belief_mean;
    cv::Mat m_belief_cov;
    double m_motions_noise_stds[4];
    double m_distance_dev_rate;
    double m_direction_dev;
        
        
PYBIND11_MODULE(_kf, m) {
    m.doc() = "pybind11";
    py::class_<KalmanFilter>(m, "KalmanFilter")
        .def(py::init<>())
    ;
}
    
    
        
         
        
        
