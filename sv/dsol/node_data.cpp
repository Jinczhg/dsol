#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sv/dsol/extra.h"
#include "sv/dsol/node_util.h"
#include "sv/ros1/msg_conv.h"
#include "sv/util/dataset.h"
#include "sv/util/logging.h"
#include "sv/util/ocv.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace sv::dsol {

using SE3d = Sophus::SE3d;
namespace gm = geometry_msgs;
namespace sm = sensor_msgs;
namespace vm = visualization_msgs;

struct NodeData {
  explicit NodeData(const ros::NodeHandle& pnh);

  void InitOdom();
  void InitRosIO();
  void InitDataset();
  void InitPcdFile();

  void PublishOdom(const std_msgs::Header& header, const Sophus::SE3d& Twc);
  void PublishCloud(const std_msgs::Header& header) const;
  void PublishImageL(const std_msgs::Header& header, auto imageL) const;
  void PublishImageR(const std_msgs::Header& header, auto imageR) const;
  void SendTransform(const gm::PoseStamped& pose_msg,
                     const std::string& child_frame);
  
  void SavePointCloud(sensor_msgs::PointCloud2& cloud) const;
  void Run();

  bool reverse_{false};
  double freq_{10.0};
  double data_max_depth_{0};
  double cloud_max_depth_{150};
  cv::Range data_range_{0, 0};
  
  std::string pcdFileName;
  std::ofstream pcdFile;
  mutable int ptCnt;

  Dataset dataset_;
  MotionModel motion_;
  TumFormatWriter writer_;
  DirectOdometry odom_;

  KeyControl ctrl_;
  std::string frame_{"fixed"};
  tf2_ros::TransformBroadcaster tfbr_;

  ros::NodeHandle pnh_;
  ros::Publisher clock_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher align_marker_pub_;
  PosePathPublisher gt_pub_;
  PosePathPublisher kf_pub_;
  PosePathPublisher odom_pub_;

  ros::Publisher points_pub_;
  
  ros::Publisher image_l_pub_;
  ros::Publisher image_r_pub_;
};

NodeData::NodeData(const ros::NodeHandle& pnh) : pnh_{pnh} {
  InitRosIO();
  InitDataset();
  InitOdom();
  InitPcdFile();

  const int wait_ms = pnh_.param<int>("wait_ms", 0);
  ROS_INFO_STREAM("wait_ms: " << wait_ms);
  ctrl_ = KeyControl(wait_ms);

  const auto save = pnh_.param<std::string>("save", "");
  writer_ = TumFormatWriter(save);
  if (!writer_.IsDummy()) {
    ROS_WARN_STREAM("Writing results to: " << writer_.filename());
  }

  const auto alpha = pnh_.param<double>("motion_alpha", 0.5);
  motion_ = MotionModel(alpha);
  ROS_INFO_STREAM("motion_alpha: " << motion_.alpha());

  // this is to make camera z pointing forward
  //  const Eigen::Quaterniond q_f_c0(
  //      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
  //      Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
  //  T_f_c0_.setQuaternion(q_f_c0);
  //  ROS_INFO_STREAM("T_f_c0: \n" << T_f_c0_.matrix());
}

void NodeData::InitRosIO() {
  clock_pub_ = pnh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  gt_pub_ = PosePathPublisher(pnh_, "gt", frame_);
  kf_pub_ = PosePathPublisher(pnh_, "kf", frame_);
  odom_pub_ = PosePathPublisher(pnh_, "odom", frame_);
  points_pub_ = pnh_.advertise<sm::PointCloud2>("points", 1);
  pose_array_pub_ = pnh_.advertise<gm::PoseArray>("poses", 1);
  align_marker_pub_ = pnh_.advertise<vm::Marker>("align_graph", 1);
  image_l_pub_ = pnh_.advertise<sm::Image>("image_l", 1);
  image_r_pub_ = pnh_.advertise<sm::Image>("image_r", 1);
}

void NodeData::InitDataset() {
  const auto data_dir = pnh_.param<std::string>("data_dir", {});
  dataset_ = CreateDataset(data_dir);
  if (!dataset_) {
    ROS_ERROR_STREAM("Invalid dataset at: " << data_dir);
    ros::shutdown();
  }
  ROS_INFO_STREAM(dataset_.Repr());

  pnh_.getParam("start", data_range_.start);
  pnh_.getParam("end", data_range_.end);
  pnh_.getParam("reverse", reverse_);

  if (data_range_.end <= 0) {
    data_range_.end += dataset_.size();
  }
  ROS_INFO("Data range: [%d, %d)", data_range_.start, data_range_.end);
  ROS_INFO("Reverse: %s", reverse_ ? "true" : "false");

  pnh_.getParam("freq", freq_);
  pnh_.getParam("data_max_depth", data_max_depth_);
  pnh_.getParam("cloud_max_depth", cloud_max_depth_);

  ROS_INFO_STREAM("Freq: " << freq_);
  ROS_INFO_STREAM("Max depth: " << data_max_depth_);
}

void NodeData::InitOdom() {
  {
    auto cfg = ReadOdomCfg({pnh_, "odom"});
    pnh_.getParam("tbb", cfg.tbb);
    pnh_.getParam("log", cfg.log);
    pnh_.getParam("vis", cfg.vis);
    odom_.Init(cfg);
  }
  odom_.selector = PixelSelector(ReadSelectCfg({pnh_, "select"}));
  odom_.matcher = StereoMatcher(ReadStereoCfg({pnh_, "stereo"}));
  odom_.aligner = FrameAligner(ReadDirectCfg({pnh_, "align"}));
  odom_.adjuster = BundleAdjuster(ReadDirectCfg({pnh_, "adjust"}));
  odom_.cmap = GetColorMap(pnh_.param<std::string>("cm", "jet"));

  ROS_INFO_STREAM(odom_.Repr());
}

void NodeData::InitPcdFile() {
  
  pcdFileName = "/home/jzhang72/NetBeansProjects/ROS_Fast_DSO/ros_ws/output/pointCloud_dsol.pcd";
  pcdFile.open(pcdFileName);
  ptCnt = 0;
  const int countSpace = 19;
  if (boost::filesystem::exists(pcdFileName)) {
    
    pcdFile << R"__(# .PCD v0.6 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F U
COUNT 1 1 1 1
WIDTH 0)__" +
            std::string(countSpace - 1, ' ') +
            R"__(
HEIGHT 1
#VIEWPOINT 0 0 0 1 0 0 0
POINTS 0)__" +
            std::string(countSpace - 1, ' ') +
            R"__(
DATA ascii
)__";
    
  }
  pcdFile.flush();
  pcdFile.close();
}

void NodeData::PublishCloud(const std_msgs::Header& header) const {
  if (points_pub_.getNumSubscribers() == 0) return;

  static sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.point_step = 16;
  cloud.fields = MakePointFields("xyzi");

  ROS_DEBUG_STREAM(odom_.window.MargKf().status().Repr());
  Keyframe2Cloud(odom_.window.MargKf(), cloud, cloud_max_depth_);
  points_pub_.publish(cloud);
  this->SavePointCloud(cloud);
}

void NodeData::SavePointCloud(sensor_msgs::PointCloud2& cloud) const {
      
  // save point cloud to a PCD file
  std::ofstream pcd;
  const int countSpace = 19;
  const int countPos = 118;
  const int countPos2 = 179;
  if (boost::filesystem::exists(pcdFileName)) {
    pcd.open(pcdFileName, std::ios_base::app);
    for(int i = 0 ; i < cloud.width * cloud.height; ++i){
       auto* ptr = reinterpret_cast<float*>(cloud.data.data() + i * cloud.point_step);
       if (std::isnan(ptr[0]) or std::isnan(ptr[1]) or std::isnan(ptr[2]))
           continue;
       ptCnt++;
       ptr[3] = ptr[3]*255.0;
       uint32_t rgb = ((uint32_t) ptr[3] << 16 | (uint32_t) ptr[3] << 8 | (uint32_t) ptr[3]);
       pcd << ptr[0] << " " << ptr[1] << " " << ptr[2] << " " << rgb << "\n";
    }
    std::cout << "Writing " << ptCnt << " points to the PCD file." << std::endl;
    // Update the total points in the PCD file every time.
    std::fstream fs(pcdFileName);
    std::string emplacedValue = std::to_string(ptCnt);
    emplacedValue += std::string(countSpace - emplacedValue.length(), ' ');
    fs.seekp(countPos) << emplacedValue;
    fs.seekp(countPos2) << emplacedValue;
    fs.close();
  }
  pcd.flush();
  pcd.close();
}

void NodeData::PublishImageL(const std_msgs::Header& header, auto image_l) const {
  if (image_l_pub_.getNumSubscribers() == 0) return;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image_l).toImageMsg();
  image_l_pub_.publish(msg);
}

void NodeData::PublishImageR(const std_msgs::Header& header, auto image_r) const {
  if (image_r_pub_.getNumSubscribers() == 0) return;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image_r).toImageMsg();
  image_r_pub_.publish(msg);
}

void NodeData::SendTransform(const geometry_msgs::PoseStamped& pose_msg,
                             const std::string& child_frame) {
  gm::TransformStamped tf_msg;
  tf_msg.header = pose_msg.header;
  tf_msg.child_frame_id = child_frame;
  Ros2Ros(pose_msg.pose, tf_msg.transform);
  tfbr_.sendTransform(tf_msg);
}

void NodeData::Run() {
  ros::Time time{};
  const auto dt = 1.0 / freq_;
  const ros::Duration dtime{ros::Rate{freq_}};

  bool init_tf{false};
  SE3d T_c0_w_gt;
  SE3d dT_pred;

  int start_ind = reverse_ ? data_range_.end - 1 : data_range_.start;
  int end_ind = reverse_ ? data_range_.start - 1 : data_range_.end;
  const int delta = reverse_ ? -1 : 1;

  // Marker
  vm::Marker align_marker;

  for (int ind = start_ind, cnt = 0; ind != end_ind; ind += delta, ++cnt) {
    if (!ros::ok() || !ctrl_.Wait()) break;

    ROS_INFO("=== %d ===", ind);
    rosgraph_msgs::Clock clock;
    clock.clock = time;
    clock_pub_.publish(clock);

    // Image
    auto image_l = dataset_.Get(DataType::kImage, ind, 0);
    auto image_r = dataset_.Get(DataType::kImage, ind, 1);

    // Intrin
    if (!odom_.camera.Ok()) {
      const auto intrin = dataset_.Get(DataType::kIntrin, ind);
      const auto camera = Camera::FromMat({image_l.cols, image_l.rows}, intrin);
      odom_.SetCamera(camera);
      ROS_INFO_STREAM(camera);
    }

    // Depth
    auto depth = dataset_.Get(DataType::kDepth, ind, 0);

    if (!depth.empty()) {
      if (data_max_depth_ > 0) {
        cv::threshold(depth, depth, data_max_depth_, 0, cv::THRESH_TOZERO_INV);
      }
    }

    // Pose
    const auto pose_gt = dataset_.Get(DataType::kPose, ind, 0);

    // Record the inverse of the first transform
    if (!init_tf) {
      T_c0_w_gt = SE3dFromMat(pose_gt).inverse();
      ROS_INFO_STREAM("T_c0_w:\n" << T_c0_w_gt.matrix());
      init_tf = true;
    }

    // Then we transform everything into c0 frame
    const auto T_c0_c_gt = T_c0_w_gt * SE3dFromMat(pose_gt);

    // Motion model predict
    if (!motion_.Ok()) {
      motion_.Init(T_c0_c_gt);
    } else {
      dT_pred = motion_.PredictDelta(dt);
    }

    const auto T_pred = odom_.frame.Twc() * dT_pred;

    // Odom
    const auto status = odom_.Estimate(image_l, image_r, dT_pred, depth);
    ROS_INFO_STREAM(status.Repr());

    // Motion model correct if tracking is ok and not first frame
    if (status.track.ok && ind != start_ind) {
      motion_.Correct(status.Twc(), dt);
    } else {
      ROS_WARN_STREAM("Tracking failed (or 1st frame), slow motion model");
      motion_.Scale(0.5);
    }
    
    // Write depth data
//    float sumDepth = 0;
//    float validPixel = 0;
//    const Keyframe& keyframe = odom_.window.MargKf();
//    const auto& points = keyframe.points();
//    for (int gr = 0; gr < points.rows(); ++gr) {
//      for (int gc = 0; gc < points.cols(); ++gc) {
//        const auto& point = points.at(gr, gc);
//        // Only draw points with max info and within max depth
//        if (!point.InfoMax() || (1.0 / point.idepth()) > cloud_max_depth_) {
//          continue;
//        }
//        sumDepth += (float)point.pt().z();
//        validPixel++;
//      }
//    }
    
//    if (validPixel > 0) {
//        float avgDepth = sumDepth / validPixel;
//        std::ofstream file;
//        file.open("avg_frame_depth.txt", std::ios::app | std::ios::out);
//        if (file.is_open())
//            file << avgDepth << std::endl;
//        file.close();
//    }

    // Write to output
    writer_.Write(cnt, status.Twc());
    
    ROS_DEBUG_STREAM("trans gt:   " << T_c0_c_gt.translation().transpose());
    ROS_DEBUG_STREAM("trans pred: " << T_pred.translation().transpose());
    ROS_DEBUG_STREAM("trans odom: " << status.Twc().translation().transpose());
    ROS_DEBUG_STREAM("trans ba:   "
                     << odom_.window.CurrKf().Twc().translation().transpose());
    ROS_DEBUG_STREAM("aff_l: " << odom_.frame.state().affine_l.ab.transpose());
    ROS_DEBUG_STREAM("aff_r: " << odom_.frame.state().affine_r.ab.transpose());

    // publish stuff
    std_msgs::Header header;
    header.frame_id = frame_;
    header.stamp = time;

    gt_pub_.Publish(time, T_c0_c_gt);
    PublishOdom(header, status.Twc());

    if (status.map.remove_kf) {
      PublishCloud(header);
      PublishImageL(header, image_l);
      PublishImageR(header, image_r);
    }
    
    // Draw align graph
    //    align_marker.header = header;
    //    DrawAlignGraph(status.Twc().translation(),
    //                   odom_.window.GetAllTrans(),
    //                   odom_.aligner.num_tracks(),
    //                   CV_RGB(1.0, 0.0, 0.0),
    //                   0.1,
    //                   align_marker);
    //    align_marker_pub_.publish(align_marker);

    time += dtime;
  }
}

void NodeData::PublishOdom(const std_msgs::Header& header,
                           const Sophus::SE3d& Twc) {
  const auto odom_pose_msg = odom_pub_.Publish(header.stamp, Twc);
  SendTransform(odom_pose_msg, "camera");

  const auto poses = odom_.window.GetAllPoses();
  gm::PoseArray pose_array_msg;
  pose_array_msg.header = header;
  pose_array_msg.poses.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    Sophus2Ros(poses.at(i), pose_array_msg.poses.at(i));
  }
  pose_array_pub_.publish(pose_array_msg);
}

}  // namespace sv::dsol

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsol_data");
  sv::dsol::NodeData node{ros::NodeHandle{"~"}};
  node.Run();
}
