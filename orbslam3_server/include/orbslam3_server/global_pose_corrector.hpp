#pragma once

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "orbslam3_msgs/msg/map_correction.hpp"
#include "orbslam3_msgs/msg/corrected_key_frame_array.hpp"
#include "orbslam3_msgs/msg/corrected_key_frame.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <unordered_map>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>

struct CorrectedKeyFrameData
{
    uint64_t keyframe_id;

    geometry_msgs::msg::Pose original_local_pose;
    geometry_msgs::msg::Pose original_global_pose;
    geometry_msgs::msg::Pose corrected_global_pose;
};

struct PoseCorrectionDiagnostics
{
    std::string mode = "NO_CORRECTION";

    uint64_t selected_keyframe_id = 0;

    double nearest_kf_distance_m =
        std::numeric_limits<double>::infinity();

    double selected_kf_distance_m =
        std::numeric_limits<double>::infinity();

    double map_correction_age_s =
        std::numeric_limits<double>::infinity();

    double corrected_keyframes_age_s =
        std::numeric_limits<double>::infinity();

    double output_jump_m = 0.0;
    double output_jump_yaw_deg = 0.0;

    bool has_valid_map_correction = false;
    bool has_valid_corrected_keyframes = false;
    bool output_is_finite = false;
};

class GlobalPoseCorrector : public rclcpp::Node
{
public:
    GlobalPoseCorrector();

private:
    void LocalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void GroundTruthPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void MapCorrectionCallback(const orbslam3_msgs::msg::MapCorrection::SharedPtr msg);

    void CorrectedKeyFramesCallback(const orbslam3_msgs::msg::CorrectedKeyFrameArray::SharedPtr msg);

    tf2::Transform PoseMsgToTf2(const geometry_msgs::msg::Pose& pose);

    tf2::Transform TransformMsgToTf2(const geometry_msgs::msg::Transform& transform);

    geometry_msgs::msg::Pose Tf2ToPoseMsg(const tf2::Transform& tf);

    bool IsFiniteTransform(const tf2::Transform& tf) const;

    bool IsFinitePose(const geometry_msgs::msg::Pose& pose) const;

    double TranslationDistance(
        const tf2::Transform& a,
        const tf2::Transform& b) const;

    double YawFromTransform(const tf2::Transform& tf) const;

    double NormalizeAngleRad(double a) const;

    double YawDifferenceDeg(
        const tf2::Transform& a,
        const tf2::Transform& b) const;

    double AgeSeconds(
        const rclcpp::Time& stamp) const;

    tf2::Transform InterpolateTransform(
        const tf2::Transform& from,
        const tf2::Transform& to,
        double alpha) const;

    tf2::Quaternion NormalizeQuaternion(
        const tf2::Quaternion& q) const;

    double ClampDouble(
        double value,
        double lo,
        double hi) const;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<orbslam3_msgs::msg::MapCorrection>::SharedPtr map_correction_sub_;
    rclcpp::Subscription<orbslam3_msgs::msg::CorrectedKeyFrameArray>::SharedPtr corrected_keyframes_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub_;

    // Pose suavizada. Mantiene el tópico original para futuros consumidores.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;

    // FASE 4B:
    // Pose corregida sin suavizar. Útil para comparar contra la suavizada.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_raw_pub_;

    // ============================================================
    // FASE 4C:
    // Pose global del cuerpo del dron, no solo de la cámara.
    // Este es el tópico que debería usar un controlador futuro.
    // ============================================================

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_body_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_body_pose_raw_pub_;

    uint32_t drone_id_;
    std::string world_frame_;

    geometry_msgs::msg::Transform world_T_local_map_;
    bool has_world_T_local_map_ = false;

    std::unordered_map<uint64_t, CorrectedKeyFrameData> corrected_keyframes_;
    bool has_corrected_keyframes_ = false;

    bool use_corrected_keyframes_;
    double max_nearest_kf_distance_m_;

    // ============================================================
    // FASE 4A:
    // Diagnóstico y seguridad de pose corregida.
    // ============================================================

    // ============================================================
    // FASE 4A-FIX:
    // La map_correction puede no llegar a alta frecuencia.
    // Para control, no debemos cortar la publicación global cada vez
    // que pasan 2 s sin nueva corrección.
    //
    // Usamos dos umbrales:
    // - warning age: publicamos, pero avisamos.
    // - hard timeout: dejamos de publicar por seguridad.
    // ============================================================

    double map_correction_stale_warn_age_s_ = 2.0;
    double map_correction_hard_timeout_s_ = 30.0;

    double max_corrected_keyframes_age_s_ = 5.0;

    bool publish_using_last_correction_when_stale_ = true;
    bool allow_rigid_fallback_when_corrected_kfs_stale_ = true;

    double warn_output_jump_translation_m_ = 0.75;
    double warn_output_jump_yaw_deg_ = 25.0;

    rclcpp::Time last_map_correction_rx_time_;
    rclcpp::Time last_corrected_keyframes_rx_time_;
    rclcpp::Time last_local_pose_rx_time_;

    bool has_last_published_pose_ = false;
    tf2::Transform last_published_world_T_camera_;

    // ============================================================
    // FASE 4B:
    // Suavizado y limitación de saltos para pose usable en control.
    //
    // Regla:
    // - raw pose: pose corregida directa.
    // - smoothed pose: pose corregida suavizada.
    // ============================================================

    bool enable_pose_smoothing_ = true;

    // Si true, el tópico pose_global_corrected publica la pose suavizada.
    // La raw se publica en pose_global_corrected_raw.
    bool publish_smoothed_pose_on_main_topic_ = true;

    // Factor base de interpolación.
    // 1.0 = sin suavizado.
    // 0.0 = no moverse.
    double pose_smoothing_alpha_ = 0.25;

    // Límites máximos aplicados por actualización al suavizado.
    double max_smoothed_translation_step_m_ = 0.20;
    double max_smoothed_yaw_step_deg_ = 8.0;

    // Si el salto raw es enorme, reiniciamos el suavizado en vez de arrastrarlo.
    // Esto es útil ante resets de ORB-SLAM3 o cambios de epoch.
    double reset_smoothing_if_raw_jump_m_ = 8.0;
    double reset_smoothing_if_raw_jump_yaw_deg_ = 120.0;

    // Si no publicamos durante mucho tiempo, reiniciamos suavizado.
    double reset_smoothing_if_no_publish_s_ = 2.0;

    bool has_smoothed_pose_ = false;
    tf2::Transform smoothed_world_T_camera_;

    rclcpp::Time last_smoothed_publish_time_;

    // ============================================================
    // FASE 4C:
    // Transformación fija body_T_camera.
    // Ojo: no es solo traslación. Aunque la cámara no tenga giro
    // mecánico, el frame óptico de cámara suele tener ejes distintos.
    // ============================================================

    bool publish_body_pose_ = true;

    double body_T_camera_x_ = 0.10;
    double body_T_camera_y_ = 0.03;
    double body_T_camera_z_ = 0.03;

    double body_T_camera_roll_deg_ = 0.0;
    double body_T_camera_pitch_deg_ = 0.0;
    double body_T_camera_yaw_deg_ = 0.0;

    // FASE 4C-FIX:
    // Si true, ignoramos body_T_camera_rpy_deg y usamos la convención
    // óptica estándar:
    //   camera_z = body_x
    //   camera_x = -body_y
    //   camera_y = -body_z
    bool use_camera_optical_frame_convention_ = true;

    tf2::Transform body_T_camera_;
    tf2::Transform camera_T_body_;

    tf2::Quaternion BuildBodyTCameraQuaternion() const;

    bool enable_gt_pose_diagnostics_ = true;

    bool has_gt_body_pose_ = false;
    geometry_msgs::msg::PoseStamped last_gt_body_pose_;

    double warn_body_position_error_m_ = 0.75;
    double warn_body_yaw_error_deg_ = 20.0;
};