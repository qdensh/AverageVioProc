#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <fstream>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
using namespace std;

// get_filenames
int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
{
	fs::path path(dir);
	if (!fs::exists(path))
	{
		return -1;
	}
 
	fs::directory_iterator end_iter;
	for (fs::directory_iterator iter(path); iter!=end_iter; ++iter)
	{
		if (fs::is_regular_file(iter->status()))
		{
			filenames.push_back(iter->path().string());
		}
 
		if (fs::is_directory(iter->status()))
		{
			get_filenames(iter->path().string(), filenames);
		}
	}
 
	return filenames.size();
}

// del_filenames
int del_filenames(const std::string& dir)
{
	fs::path path(dir);
	if (!fs::exists(path))
	{
		return -1;
	}
 
	fs::directory_iterator end_iter;
	for (fs::directory_iterator iter(path); iter!=end_iter; ++iter)
	{
		if (fs::is_regular_file(iter->status()))
		{
			fs::remove(iter->path());
		}
 
		if (fs::is_directory(iter->status()))
		{
			del_filenames(iter->path().string());
		}
	}

  return 0;
}

std::string output_path;
std::string input_odom_path;
std::ofstream pose_odom_file;
std::ofstream pose_gt_file;
std::ofstream odom_file;
bool is_average_output_odom = false;

// CSVDATA_ODOM
struct CSVDATA_ODOM {
  double time;
  double Dtime;
  Eigen::Vector3d pB;
  Eigen::Vector4d qB;
  Eigen::Vector4d qB_G;
  Eigen::Vector3d vB;
  Eigen::Vector3d wB;

  Eigen::Vector3d gt_pB;
  Eigen::Vector4d gt_qB;
  Eigen::Vector4d gt_qB_G;
  Eigen::Vector3d gt_vB;
  Eigen::Vector3d gt_wB;

  double Sr;
  double Sr_avg;
  double Sr_rms;

  long long int numlines;

  Eigen::Vector3d pIBs;
  Eigen::Vector4d qIBs;

  Eigen::Matrix<double, 5, 1> k_b;

};

// align_odom
double freq_time_odom = 0;
void align_odom(std::vector<struct CSVDATA_ODOM> & csvData1, std::vector<struct CSVDATA_ODOM> &csvData2){
  std::vector<struct CSVDATA_ODOM>  csvData1_tmp;
  std::vector<struct CSVDATA_ODOM>  csvData2_tmp;
  for(auto csvdata1 : csvData1){

    double min_dtime = csvdata1.time;
    bool flag_has_min = false;
    struct CSVDATA_ODOM csvdata2_min;

    for(auto csvdata2 : csvData2){

      double dtime = abs(csvdata1.time - csvdata2.time);
      
      if( dtime < (freq_time_odom  - 0.005) ){

        if( dtime < min_dtime){
          min_dtime = dtime;

          csvdata2_min = csvdata2;
          flag_has_min = true;
        }
      }  

    }

    if(flag_has_min)
    {
      csvData1_tmp.push_back(csvdata1);
      csvData2_tmp.push_back(csvdata2_min);
    }

  }
  csvData1 = csvData1_tmp;
  csvData2 = csvData2_tmp;
}

// do_average_output_odom
void do_average_output_odom(){

      std::vector<std::string> filenames;
      int num_files = get_filenames(input_odom_path, filenames);
      if(num_files <= 0){
        ROS_INFO_STREAM("do_average_output_odom: filenames.size() == 0");
        return;
      }
      cout << "do_average_output_odom num_files: " << num_files << endl;

      string file_time;
      [&]() { 
          time_t timep;
            time (&timep);
            char tmp[64];
            strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
            file_time = tmp;
          }();

      cout.precision(16);
      std::vector<std::vector<struct CSVDATA_ODOM>> csvDataBus;
      int lineArray_length = 0;
      for(auto & filename : filenames){
        ifstream inFile(filename.c_str(), std::ios::in);
        if (!inFile)
        {
            cout << "read csv file error：can not open" << endl;
            return;
        }

        bool flag_first = true;
        double start_time = 0;
        double end_time = 0;
        double avg_time = 0;
        long long int count  = 0;
        double Sr_sum_square = 0;

        std::string lineStr;
        std::vector<struct CSVDATA_ODOM> csvData;
        std::getline(inFile, lineStr); 
        while (std::getline(inFile, lineStr)) {
            std::stringstream ss(lineStr); 
            std::string str;
            vector<string> lineArray;
            while (getline(ss, str, ',')) {
                lineArray.push_back(str);
            }
            lineArray_length = lineArray.size();
            struct CSVDATA_ODOM csvdata;
            csvdata.time = stod(lineArray[0]);
            csvdata.Dtime = stod(lineArray[1]);
            csvdata.pB = {stod(lineArray[3]),stod(lineArray[4]),stod(lineArray[5])};
            csvdata.qB = {stod(lineArray[6]),stod(lineArray[7]),stod(lineArray[8]),stod(lineArray[9])};
            
            if(flag_first){
              start_time = csvdata.time;
              flag_first = false;
            }
            end_time = csvdata.time;
            count++;

            if(lineArray_length > 10){

              geometry_msgs::Quaternion q_rpy_G = tf::createQuaternionMsgFromRollPitchYaw(stod(lineArray[10]) * M_PI / 180, 
                stod(lineArray[11]) * M_PI / 180, stod(lineArray[12]) * M_PI / 180);
              csvdata.qB_G = {q_rpy_G.x, q_rpy_G.y, q_rpy_G.z, q_rpy_G.w};

              csvdata.vB = {stod(lineArray[13]),stod(lineArray[14]),stod(lineArray[15])};
              csvdata.wB = {stod(lineArray[16]),stod(lineArray[17]),stod(lineArray[18])};

              csvdata.gt_pB = {stod(lineArray[20]),stod(lineArray[21]),stod(lineArray[22])};
              csvdata.gt_qB = {stod(lineArray[23]),stod(lineArray[24]),stod(lineArray[25]),stod(lineArray[26])};

              geometry_msgs::Quaternion gt_q_rpy_G = tf::createQuaternionMsgFromRollPitchYaw(stod(lineArray[27]) * M_PI / 180, 
                stod(lineArray[28]) * M_PI / 180, stod(lineArray[29]) * M_PI / 180);
              csvdata.gt_qB_G = {gt_q_rpy_G.x, gt_q_rpy_G.y, gt_q_rpy_G.z, gt_q_rpy_G.w};

              csvdata.gt_vB = {stod(lineArray[30]),stod(lineArray[31]),stod(lineArray[32])};
              csvdata.gt_wB = {stod(lineArray[33]),stod(lineArray[34]),stod(lineArray[35])};

              csvdata.Sr = stod(lineArray[37]);
              csvdata.Sr_avg = stod(lineArray[38]);


              Sr_sum_square += csvdata.Sr * csvdata.Sr;
              double Sr_sum_square_avg = Sr_sum_square / count;
              csvdata.Sr_rms = sqrt(Sr_sum_square_avg);

            }
            
            csvData.push_back(csvdata);
        }
        csvDataBus.push_back(csvData);

        if(count > 1){
          avg_time = (end_time - start_time) / (count - 1);
        }
        freq_time_odom += avg_time;

      }
      freq_time_odom /= num_files;
      cout << "freq_time_odom(ms): " << freq_time_odom * 1000 << endl;

      for(int j = 1; j < csvDataBus.size(); j++){
        // align_odom
        align_odom(csvDataBus.at(0),csvDataBus.at(j));
      }
      for(int j = 1; j < csvDataBus.size(); j++){
        // align_odom again
        align_odom(csvDataBus.at(0),csvDataBus.at(j));
      }
      for(int j = 1; j < csvDataBus.size(); j++){
        if(csvDataBus.at(0).size() != csvDataBus.at(j).size()){
          ROS_INFO_STREAM("do_average_output_odom: csvDataBus.at(0).size() != csvDataBus.at(j).size()");
          return;
        }
      }

      std::string delim = ",";
      // pose odom
      pose_odom_file.open((output_path + "pose_odom_avg" + "_" + file_time + ".txt"), std::ios::out);
      if(!pose_odom_file.is_open())
      {
          cerr << "pose_odom_avg is not open" << endl;
      }

      // pose gt
      pose_gt_file.open((output_path + "pose_gt_avg" + "_" + file_time + ".txt"), std::ios::out);
      if(!pose_gt_file.is_open())
      {
          cerr << "pose_gt_avg is not open" << endl;
      }

      // odom
      odom_file.open((output_path + "odom_avg" + "_" + file_time + ".csv"), std::ios::out);
      if(!odom_file.is_open())
      {
          cerr << "odom_avg is not open" << endl;
      }
      if(lineArray_length > 10){
          odom_file << "#Time(sec),";
          odom_file << "Dtime(sec),";
          odom_file  << delim;
          odom_file << "x(m),y(m),z(m),";
          odom_file << "qx,qy,qz,qw,";
          odom_file << "roll_x_G(deg),pitch_y_G(deg),yaw_z_G(deg),";
          odom_file << "vx(m/s),vy(m/s),vz(m/s),";
          odom_file << "wx(rad/s),wy(rad/s),wz(rad/s),";
          odom_file  << delim;
          odom_file << "gt_x(m),gt_y(m),gt_z(m),";
          odom_file << "gt_qx,gt_qy,gt_qz,gt_qw,";
          odom_file << "gt_roll_x_G(deg),gt_pitch_y_G(deg),gt_yaw_z_G(deg),";
          odom_file << "gt_vx(m/s),gt_vy(m/s),gt_vz(m/s),";
          odom_file << "gt_wx(rad/s),gt_wy(rad/s),gt_wz(rad/s),";
          odom_file  << delim;
          odom_file << "Sr,Sr_avg,Sr_rms_avg,";
          odom_file << std::endl;
        
      }else{
        odom_file << "#Time(sec),";
        odom_file << "Dtime(sec),";
        odom_file  << delim;
        odom_file << "x(m),y(m),z(m),";
        odom_file << "qx,qy,qz,qw,";
        odom_file << std::endl;
      }
      
      std::vector<std::vector<Eigen::Vector3d>> pB_bus;
      std::vector<std::vector<Eigen::Vector4d>> qB_bus;
      std::vector<std::vector<Eigen::Vector4d>> qB_bus_G;
      std::vector<std::vector<Eigen::Vector3d>> vB_bus;
      std::vector<std::vector<Eigen::Vector3d>> wB_bus;
      std::vector<std::vector<Eigen::Vector3d>> gt_pB_bus;
      std::vector<std::vector<Eigen::Vector4d>> gt_qB_bus;
      std::vector<std::vector<Eigen::Vector4d>> gt_qB_bus_G;
      std::vector<std::vector<Eigen::Vector3d>> gt_vB_bus;
      std::vector<std::vector<Eigen::Vector3d>> gt_wB_bus;
      std::vector<std::vector<double>> Sr_bus;
      std::vector<std::vector<double>> Sr_avg_bus;
      std::vector<std::vector<double>> Sr_rms_bus;

      std::vector<std::vector<Eigen::Vector3d>> pIBs_bus;
      std::vector<std::vector<Eigen::Vector4d>> qIBs_bus;

      std::vector<std::vector<Eigen::Matrix<double, 5, 1>>> k_b_bus;

      std::vector<struct CSVDATA_ODOM> first_csv_bus = csvDataBus.at(0);
      int num_lines = first_csv_bus.size();
      for(int i = 0; i < num_lines; i++){
        std::vector<Eigen::Vector3d> pB_bus_data;
        std::vector<Eigen::Vector4d> qB_bus_data;
        std::vector<Eigen::Vector4d> qB_bus_data_G;
        std::vector<Eigen::Vector3d> vB_bus_data;
        std::vector<Eigen::Vector3d> wB_bus_data;
        std::vector<Eigen::Vector3d> gt_pB_bus_data;
        std::vector<Eigen::Vector4d> gt_qB_bus_data;
        std::vector<Eigen::Vector4d> gt_qB_bus_data_G;
        std::vector<Eigen::Vector3d> gt_vB_bus_data;
        std::vector<Eigen::Vector3d> gt_wB_bus_data;
        std::vector<double> Sr_bus_data;
        std::vector<double> Sr_avg_bus_data;
        std::vector<double> Sr_rms_bus_data;

        std::vector<Eigen::Vector3d> pIBs_bus_data;
        std::vector<Eigen::Vector4d> qIBs_bus_data;

        std::vector<Eigen::Matrix<double, 5, 1>> k_b_bus_data;

        for(int j = 0; j < num_files; j++){
          std::vector<struct CSVDATA_ODOM> csvdata = csvDataBus.at(j);
          pB_bus_data.push_back(csvdata.at(i).pB);
          qB_bus_data.push_back(csvdata.at(i).qB);
          vB_bus_data.push_back(csvdata.at(i).vB);
          wB_bus_data.push_back(csvdata.at(i).wB);
          gt_pB_bus_data.push_back(csvdata.at(i).gt_pB);
          gt_qB_bus_data.push_back(csvdata.at(i).gt_qB);
          gt_vB_bus_data.push_back(csvdata.at(i).gt_vB);
          gt_wB_bus_data.push_back(csvdata.at(i).gt_wB);
          Sr_bus_data.push_back(csvdata.at(i).Sr);
          Sr_avg_bus_data.push_back(csvdata.at(i).Sr_avg);
          Sr_rms_bus_data.push_back(csvdata.at(i).Sr_rms);

          pIBs_bus_data.push_back(csvdata.at(i).pIBs);
          qIBs_bus_data.push_back(csvdata.at(i).qIBs);

          k_b_bus_data.push_back(csvdata.at(i).k_b);

          qB_bus_data_G.push_back(csvdata.at(i).qB_G);
          gt_qB_bus_data_G.push_back(csvdata.at(i).gt_qB_G);

        }
        pB_bus.push_back(pB_bus_data);
        qB_bus.push_back(qB_bus_data);
        vB_bus.push_back(vB_bus_data);
        wB_bus.push_back(wB_bus_data);
        gt_pB_bus.push_back(gt_pB_bus_data);
        gt_qB_bus.push_back(gt_qB_bus_data);
        gt_vB_bus.push_back(gt_vB_bus_data);
        gt_wB_bus.push_back(gt_wB_bus_data);
        Sr_bus.push_back(Sr_bus_data);
        Sr_avg_bus.push_back(Sr_avg_bus_data);
        Sr_rms_bus.push_back(Sr_rms_bus_data);

        pIBs_bus.push_back(pIBs_bus_data);
        qIBs_bus.push_back(qIBs_bus_data);

        k_b_bus.push_back(k_b_bus_data);

        qB_bus_G.push_back(qB_bus_data_G);
        gt_qB_bus_G.push_back(gt_qB_bus_data_G);

      }

      // qB_G
      std::vector<double> weightVector;
      for(int i = 0; i < num_files; i++){
        weightVector.push_back(1.0);
      }
      std::vector<Eigen::Quaterniond> avg_qB_G;
      for(auto qB_bus_data_G : qB_bus_G){

        int idex = avg_qB_G.size();
        double first_csv_w = (first_csv_bus.at(idex).qB_G).w();
        double sign = first_csv_w / abs(first_csv_w);

        std::vector<tf::Quaternion> tf_quaternions;
        tf::Quaternion tf_Quaternion;
        for(auto Qua_data : qB_bus_data_G){
          tf_Quaternion = tf::Quaternion(Qua_data.x(),Qua_data.y(),Qua_data.z(),Qua_data.w());
          tf_quaternions.push_back(tf_Quaternion);
        }
        
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, tf_quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < tf_quaternions.size(); ++i)
        {
          tf::Quaternion quat = tf_quaternions[i] * 1.0;
          Q(0,i) = quat.x();
          Q(1,i) = quat.y();
          Q(2,i) = quat.z();
          Q(3,i) = quat.w();
        }
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
          double real = eigenvalues[i].real();
          if (real > max_value)
          {
            max_value = real;
            max_idx = i;
          }
        }
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();
        tf::Quaternion mean_orientation(
          eigenvector[0].real(),
          eigenvector[1].real(),
          eigenvector[2].real(),
          eigenvector[3].real()
        );
        // Eigen::Quaterniond w x y z
        Eigen::Quaterniond q_B(mean_orientation.w(),mean_orientation.x(),mean_orientation.y(),mean_orientation.z());

        double sign_tmp = mean_orientation.w() / abs(mean_orientation.w());
        if(sign != sign_tmp){
          q_B = Eigen::Quaterniond(-mean_orientation.w(),-mean_orientation.x(),-mean_orientation.y(),-mean_orientation.z());
        }

        q_B.normalize();
        avg_qB_G.push_back(q_B);
      }

      // gt_qB_G
      std::vector<Eigen::Quaterniond> gt_avg_qB_G;
      for(auto gt_qB_bus_data_G : gt_qB_bus_G){

        int idex = gt_avg_qB_G.size();
        double first_csv_w = (first_csv_bus.at(idex).gt_qB_G).w();
        double sign = first_csv_w / abs(first_csv_w);

        std::vector<tf::Quaternion> tf_quaternions;
        tf::Quaternion tf_Quaternion;
        for(auto Qua_data : gt_qB_bus_data_G){
          tf_Quaternion = tf::Quaternion(Qua_data.x(),Qua_data.y(),Qua_data.z(),Qua_data.w());
          tf_quaternions.push_back(tf_Quaternion);
        }
        
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, tf_quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < tf_quaternions.size(); ++i)
        {
          tf::Quaternion quat = tf_quaternions[i] * 1.0;
          Q(0,i) = quat.x();
          Q(1,i) = quat.y();
          Q(2,i) = quat.z();
          Q(3,i) = quat.w();
        }
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
          double real = eigenvalues[i].real();
          if (real > max_value)
          {
            max_value = real;
            max_idx = i;
          }
        }
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();
        tf::Quaternion mean_orientation(
          eigenvector[0].real(),
          eigenvector[1].real(),
          eigenvector[2].real(),
          eigenvector[3].real()
        );
        // Eigen::Quaterniond w x y z
        Eigen::Quaterniond q_B(mean_orientation.w(),mean_orientation.x(),mean_orientation.y(),mean_orientation.z());

        double sign_tmp = mean_orientation.w() / abs(mean_orientation.w());
        if(sign != sign_tmp){
          q_B = Eigen::Quaterniond(-mean_orientation.w(),-mean_orientation.x(),-mean_orientation.y(),-mean_orientation.z());
        }

        q_B.normalize();
        gt_avg_qB_G.push_back(q_B);
      }
      
      // avg_Sr
      std::vector<double> avg_Sr;
      double Sr_square_sum = 0;
      long long int numlines = 0;
      for(auto Sr_bus_data : Sr_bus){
        double sum_tmp = 0;
        for(auto data : Sr_bus_data){
          sum_tmp = sum_tmp + data;
        }
        double avg_tmp = sum_tmp / num_files;
        avg_Sr.push_back(avg_tmp);
      }

      // avg_Sr_rms
      std::vector<double> avg_Sr_rms;
      for(auto Sr_rms_bus_data : Sr_rms_bus){
        double sum_tmp = 0;
        for(auto data : Sr_rms_bus_data){
          sum_tmp = sum_tmp + data;
        }
        double avg_tmp = sum_tmp / num_files;
        avg_Sr_rms.push_back(avg_tmp);
      }

      std::vector<double> avg_Sr_avg;
      for(auto Sr_avg_bus_data : Sr_avg_bus){
        double sum_tmp = 0;
        for(auto data : Sr_avg_bus_data){
          sum_tmp = sum_tmp + data;
        }
        double avg_tmp = sum_tmp / num_files;
        avg_Sr_avg.push_back(avg_tmp);
      }

      std::vector<Eigen::Vector3d> avg_pB;
      for(auto pB_bus_data : pB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : pB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        avg_pB.push_back(avg_tmp);
      }

      std::vector<Eigen::Vector3d> avg_vB;
      for(auto vB_bus_data : vB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : vB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        avg_vB.push_back(avg_tmp);
      }

      std::vector<Eigen::Vector3d> avg_wB;
      for(auto wB_bus_data : wB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : wB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        avg_wB.push_back(avg_tmp);
      }

      // avg_qB
      std::vector<Eigen::Quaterniond> avg_qB;
      for(auto qB_bus_data : qB_bus){

        int idex = avg_qB.size();
        double first_csv_w = (first_csv_bus.at(idex).qB).w();
        double sign = first_csv_w / abs(first_csv_w);


        std::vector<tf::Quaternion> tf_quaternions;
        tf::Quaternion tf_Quaternion;
        for(auto Qua_data : qB_bus_data){
          tf_Quaternion = tf::Quaternion(Qua_data.x(),Qua_data.y(),Qua_data.z(),Qua_data.w());
          tf_quaternions.push_back(tf_Quaternion);
        }
        
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, tf_quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < tf_quaternions.size(); ++i)
        {
          tf::Quaternion quat = tf_quaternions[i] * 1.0;
          Q(0,i) = quat.x();
          Q(1,i) = quat.y();
          Q(2,i) = quat.z();
          Q(3,i) = quat.w();
        }
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
          double real = eigenvalues[i].real();
          if (real > max_value)
          {
            max_value = real;
            max_idx = i;
          }
        }
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();
        tf::Quaternion mean_orientation(
          eigenvector[0].real(),
          eigenvector[1].real(),
          eigenvector[2].real(),
          eigenvector[3].real()
        );
        // Eigen::Quaterniond w x y z
        Eigen::Quaterniond q_B(mean_orientation.w(),mean_orientation.x(),mean_orientation.y(),mean_orientation.z());


        double sign_tmp = mean_orientation.w() / abs(mean_orientation.w());
        if(sign != sign_tmp){
          q_B = Eigen::Quaterniond(-mean_orientation.w(),-mean_orientation.x(),-mean_orientation.y(),-mean_orientation.z());
        }

        q_B.normalize();
        avg_qB.push_back(q_B);
      }

      // gt_avg_pB
      std::vector<Eigen::Vector3d> gt_avg_pB;
      for(auto gt_pB_bus_data : gt_pB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : gt_pB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        gt_avg_pB.push_back(avg_tmp);
      }

      std::vector<Eigen::Vector3d> gt_avg_vB;
      for(auto gt_vB_bus_data : gt_vB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : gt_vB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        gt_avg_vB.push_back(avg_tmp);
      }

      std::vector<Eigen::Vector3d> gt_avg_wB;
      for(auto gt_wB_bus_data : gt_wB_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : gt_wB_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        gt_avg_wB.push_back(avg_tmp);
      }

      // gt_avg_qB
      std::vector<Eigen::Quaterniond> gt_avg_qB;
      for(auto gt_qB_bus_data : gt_qB_bus){

        int idex = gt_avg_qB.size();
        double first_csv_w = (first_csv_bus.at(idex).gt_qB).w();
        double sign = first_csv_w / abs(first_csv_w);

        std::vector<tf::Quaternion> tf_quaternions;
        tf::Quaternion tf_Quaternion;
        for(auto Qua_data : gt_qB_bus_data){
          tf_Quaternion = tf::Quaternion(Qua_data.x(),Qua_data.y(),Qua_data.z(),Qua_data.w());
          tf_quaternions.push_back(tf_Quaternion);
        }
        
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, tf_quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < tf_quaternions.size(); ++i)
        {
          tf::Quaternion quat = tf_quaternions[i] * 1.0;
          Q(0,i) = quat.x();
          Q(1,i) = quat.y();
          Q(2,i) = quat.z();
          Q(3,i) = quat.w();
        }
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
          double real = eigenvalues[i].real();
          if (real > max_value)
          {
            max_value = real;
            max_idx = i;
          }
        }
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();
        tf::Quaternion mean_orientation(
          eigenvector[0].real(),
          eigenvector[1].real(),
          eigenvector[2].real(),
          eigenvector[3].real()
        );
        // Eigen::Quaterniond w x y z
        Eigen::Quaterniond q_B(mean_orientation.w(),mean_orientation.x(),mean_orientation.y(),mean_orientation.z());
        
        double sign_tmp = mean_orientation.w() / abs(mean_orientation.w());
        if(sign != sign_tmp){
          q_B = Eigen::Quaterniond(-mean_orientation.w(),-mean_orientation.x(),-mean_orientation.y(),-mean_orientation.z());
        }

        q_B.normalize();
        gt_avg_qB.push_back(q_B);
      }

      // avg_pIBs avg_qIBs
      std::vector<Eigen::Vector3d> avg_pIBs;
      for(auto pIBs_bus_data : pIBs_bus){
        Eigen::Vector3d sum_tmp = Eigen::Vector3d::Zero();
        for(auto data : pIBs_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
        Eigen::Vector3d avg_tmp = sum_tmp / num_files;
        avg_pIBs.push_back(avg_tmp);
      }

      std::vector<Eigen::Quaterniond> avg_qIBs;
      for(auto qIBs_bus_data : qIBs_bus){

        int idex = avg_qIBs.size();
        double first_csv_w = (first_csv_bus.at(idex).qIBs).w();
        double sign = first_csv_w / abs(first_csv_w);

        std::vector<tf::Quaternion> tf_quaternions;
        tf::Quaternion tf_Quaternion;
        for(auto Qua_data : qIBs_bus_data){
          tf_Quaternion = tf::Quaternion(Qua_data.x(),Qua_data.y(),Qua_data.z(),Qua_data.w());
          tf_quaternions.push_back(tf_Quaternion);
        }
        
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, tf_quaternions.size());
        Eigen::Vector3d vec;
        for (size_t i = 0; i < tf_quaternions.size(); ++i)
        {
          tf::Quaternion quat = tf_quaternions[i] * 1.0;
          Q(0,i) = quat.x();
          Q(1,i) = quat.y();
          Q(2,i) = quat.z();
          Q(3,i) = quat.w();
        }
        Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
        auto eigenvalues = es.eigenvalues();
        size_t max_idx = 0;
        double max_value = eigenvalues[max_idx].real();
        for (size_t i = 1; i < 4; ++i)
        {
          double real = eigenvalues[i].real();
          if (real > max_value)
          {
            max_value = real;
            max_idx = i;
          }
        }
        auto eigenvector = es.eigenvectors().col(max_idx).normalized();
        tf::Quaternion mean_orientation(
          eigenvector[0].real(),
          eigenvector[1].real(),
          eigenvector[2].real(),
          eigenvector[3].real()
        );
        // Eigen::Quaterniond w x y z
        Eigen::Quaterniond q_B(mean_orientation.w(),mean_orientation.x(),mean_orientation.y(),mean_orientation.z());
        
        double sign_tmp = mean_orientation.w() / abs(mean_orientation.w());
        if(sign != sign_tmp){
          q_B = Eigen::Quaterniond(-mean_orientation.w(),-mean_orientation.x(),-mean_orientation.y(),-mean_orientation.z());
        }
        
        q_B.normalize();
        avg_qIBs.push_back(q_B);
      }

      // avg k_b
      std::vector< Eigen::Matrix<double, 5, 1> > avg_k_b;
      for(auto k_b_bus_data : k_b_bus){
        Eigen::Matrix<double, 5, 1> sum_tmp = Eigen::MatrixXd::Zero(5,1);

        for(auto data : k_b_bus_data){
          sum_tmp.noalias() = sum_tmp + data;
        }
       Eigen::Matrix<double, 5, 1> avg_tmp =  (1.0 / num_files) * sum_tmp;
        avg_k_b.push_back(avg_tmp);
      }

      assert(avg_pB.size() == avg_qB.size() == avg_vB.size() == avg_wB.size() ==
        gt_avg_pB.size() == gt_avg_qB.size() == gt_avg_vB.size() == gt_avg_wB.size() == 
        avg_Sr.size() == avg_Sr_avg.size() == avg_Sr_rms.size() == avg_pIBS.size() == avg_qIBs.size() == avg_k_b.size() ==
        avg_qB_G.size() == gt_avg_qB_G.size());
      assert(avg_pB.size() == first_csv_bus.size());
      
      ROS_INFO_STREAM("***average_output_odom Saving***");

      for(int i = 0; i < avg_pB.size(); i++){

        double dStamp = first_csv_bus.at(i).time;
        double Dtime = first_csv_bus.at(i).Dtime;
        Eigen::Vector3d avg_pB_data = avg_pB.at(i);
        Eigen::Quaterniond avg_qB_data = avg_qB.at(i);
        Eigen::Quaterniond avg_qB_data_G = avg_qB_G.at(i);
        Eigen::Vector3d avg_vB_data = avg_vB.at(i);
        Eigen::Vector3d avg_wB_data = avg_wB.at(i);
        Eigen::Vector3d gt_avg_pB_data = gt_avg_pB.at(i);
        Eigen::Quaterniond gt_avg_qB_data = gt_avg_qB.at(i);
        Eigen::Quaterniond gt_avg_qB_data_G = gt_avg_qB_G.at(i);
        Eigen::Vector3d gt_avg_vB_data = gt_avg_vB.at(i);
        Eigen::Vector3d gt_avg_wB_data = gt_avg_wB.at(i);
        double avg_Sr_data = avg_Sr.at(i);
        double avg_Sr_avg_data = avg_Sr_avg.at(i);
        double avg_Sr_rms_data = avg_Sr_rms.at(i);

        Eigen::Vector3d avg_pIBs_data = avg_pIBs.at(i);
        Eigen::Quaterniond avg_qIBs_data = avg_qIBs.at(i);

        Eigen::Matrix<double, 5, 1> avg_k_b_data = avg_k_b.at(i);

        // save pose {GB}
        // TUM #timestamp(sec) x y z q_x q_y q_z q_w
        pose_odom_file.precision(16);
        pose_odom_file << fixed << dStamp << " " << avg_pB_data(0) << " " << avg_pB_data(1) << " " << avg_pB_data(2) << " "
                << avg_qB_data.x() << " " << avg_qB_data.y() << " " << avg_qB_data.z() << " " << avg_qB_data.w() << endl;

        pose_gt_file.precision(16);
        pose_gt_file << fixed << dStamp << " " << gt_avg_pB_data(0) << " " << gt_avg_pB_data(1) << " " << gt_avg_pB_data(2) << " "
                << gt_avg_qB_data.x() << " " << gt_avg_qB_data.y() << " " << gt_avg_qB_data.z() << " " << gt_avg_qB_data.w() << endl;        

        // save odom
        if(lineArray_length > 10){
          
              std::string delim = ",";
              odom_file.precision(16);

              double roll_x_G, pitch_y_G, yaw_z_G;
              tf::Matrix3x3(tf::Quaternion(avg_qB_data_G.x(),avg_qB_data_G.y(),avg_qB_data_G.z(),avg_qB_data_G.w())).getRPY(roll_x_G, pitch_y_G, yaw_z_G, 1); 
              roll_x_G = roll_x_G * 180 / M_PI;
              pitch_y_G = pitch_y_G * 180 / M_PI;
              yaw_z_G = yaw_z_G * 180 / M_PI;
              double gt_roll_x_G, gt_pitch_y_G, gt_yaw_z_G;
              tf::Matrix3x3(tf::Quaternion(gt_avg_qB_data_G.x(),gt_avg_qB_data_G.y(),gt_avg_qB_data_G.z(),gt_avg_qB_data_G.w())).getRPY(gt_roll_x_G, gt_pitch_y_G, gt_yaw_z_G, 1);
              gt_roll_x_G = gt_roll_x_G * 180 / M_PI;
              gt_pitch_y_G = gt_pitch_y_G * 180 / M_PI;
              gt_yaw_z_G = gt_yaw_z_G * 180 / M_PI;

              odom_file << fixed << dStamp << delim << Dtime << delim;
              odom_file  << delim;
              odom_file << avg_pB_data(0) << delim << avg_pB_data(1) << delim << avg_pB_data(2) << delim;
              odom_file << avg_qB_data.x() << delim <<  avg_qB_data.y() << delim << avg_qB_data.z() << delim << avg_qB_data.w() << delim;
              odom_file << roll_x_G << delim << pitch_y_G << delim << yaw_z_G << delim;
              odom_file << avg_vB_data(0) << delim << avg_vB_data(1) << delim << avg_vB_data(2) << delim;
              odom_file << avg_wB_data(0) << delim << avg_wB_data(1) << delim << avg_wB_data(2) << delim;
              odom_file  << delim;
              odom_file << gt_avg_pB_data(0) << delim << gt_avg_pB_data(1) << delim << gt_avg_pB_data(2) << delim;
              odom_file << gt_avg_qB_data.x() << delim <<  gt_avg_qB_data.y() << delim << gt_avg_qB_data.z() << delim << gt_avg_qB_data.w() << delim;
              odom_file << gt_roll_x_G << delim << gt_pitch_y_G << delim << gt_yaw_z_G << delim;
              odom_file << gt_avg_vB_data(0) << delim << gt_avg_vB_data(1) << delim << gt_avg_vB_data(2) << delim;
              odom_file << gt_avg_wB_data(0) << delim << gt_avg_wB_data(1) << delim << gt_avg_wB_data(2) << delim;
              odom_file  << delim;
              odom_file << avg_Sr_data << delim << avg_Sr_avg_data << delim << avg_Sr_rms_data << delim;
              odom_file << std::endl;
          
        }else{
          std::string delim = ",";
          odom_file.precision(16);
          odom_file << fixed << dStamp << delim << Dtime << delim;
          odom_file  << delim;
          odom_file << avg_pB_data(0) << delim << avg_pB_data(1) << delim << avg_pB_data(2) << delim;
          odom_file << avg_qB_data.x() << delim <<  avg_qB_data.y() << delim << avg_qB_data.z() << delim << avg_qB_data.w() << delim;
          odom_file << std::endl;
        }
        
      }
      pose_odom_file.close();
      pose_gt_file.close();
      odom_file.close();
      ROS_INFO_STREAM("***average_output_odom Done***");

      // // del input_odom_path
      // if(del_filenames(input_odom_path) == 0){
      //   ROS_INFO_STREAM("***del input_odom_path Done***");
      // }
      
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "average_output");
  ros::NodeHandle pnh("~");
  
  pnh.param<string>("output_path",output_path,"");
  ROS_INFO_STREAM("Loaded output_path: " << output_path);

  pnh.param<string>("input_odom_path",input_odom_path,"");
  ROS_INFO_STREAM("Loaded input_odom_path: " << input_odom_path);

  pnh.param<bool>("is_average_output_odom", is_average_output_odom, false);
  
  // thread
  std::thread thread_do_average_output_odom;

  if(is_average_output_odom){
     thread_do_average_output_odom = std::thread(do_average_output_odom);
     thread_do_average_output_odom.join();
  }

  pnh.shutdown();
  return 0;
}










