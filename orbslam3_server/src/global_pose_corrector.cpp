#include "orbslam3_server/global_pose_corrector.hpp"

GlobalPoseCorrector::GlobalPoseCorrector(): Node("global_pose_corrector")
{
    drone_id_ = this->declare_parameter<int>("drone_id", 1);
    world_frame_ = this->declare_parameter<std::string>("world_frame", "fiducial_map");

    use_corrected_keyframes_ = this->declare_parameter<bool>("use_corrected_keyframes", true);

    max_nearest_kf_distance_m_ = this->declare_parameter<double>("max_nearest_kf_distance_m", 2.0);

    map_correction_stale_warn_age_s_ =
        this->declare_parameter<double>(
            "map_correction_stale_warn_age_s",
            2.0);

    map_correction_hard_timeout_s_ =
        this->declare_parameter<double>(
            "map_correction_hard_timeout_s",
            30.0);

    max_corrected_keyframes_age_s_ =
        this->declare_parameter<double>(
            "max_corrected_keyframes_age_s",
            5.0);

    publish_using_last_correction_when_stale_ =
        this->declare_parameter<bool>(
            "publish_using_last_correction_when_stale",
            true);

    allow_rigid_fallback_when_corrected_kfs_stale_ =
        this->declare_parameter<bool>(
            "allow_rigid_fallback_when_corrected_kfs_stale",
            true);

    warn_output_jump_translation_m_ =
        this->declare_parameter<double>(
            "warn_output_jump_translation_m",
            0.75);

    warn_output_jump_yaw_deg_ =
        this->declare_parameter<double>(
            "warn_output_jump_yaw_deg",
            25.0);

    if (map_correction_stale_warn_age_s_ <= 0.0)
        map_correction_stale_warn_age_s_ = 2.0;

    if (map_correction_hard_timeout_s_ <= map_correction_stale_warn_age_s_)
        map_correction_hard_timeout_s_ = std::max(
            30.0,
            map_correction_stale_warn_age_s_ + 5.0);

    if (max_corrected_keyframes_age_s_ <= 0.0)
        max_corrected_keyframes_age_s_ = 5.0;

    if (max_nearest_kf_distance_m_ <= 0.0)
        max_nearest_kf_distance_m_ = 2.0;

    if (warn_output_jump_translation_m_ <= 0.0)
        warn_output_jump_translation_m_ = 0.75;

    if (warn_output_jump_yaw_deg_ <= 0.0)
        warn_output_jump_yaw_deg_ = 25.0;

    last_map_correction_rx_time_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    last_corrected_keyframes_rx_time_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    last_local_pose_rx_time_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    // ============================================================
    // FASE 4B:
    // Suavizado de pose global corregida.
    // ============================================================

    enable_pose_smoothing_ =
        this->declare_parameter<bool>(
            "enable_pose_smoothing",
            true);

    publish_smoothed_pose_on_main_topic_ =
        this->declare_parameter<bool>(
            "publish_smoothed_pose_on_main_topic",
            true);

    pose_smoothing_alpha_ =
        this->declare_parameter<double>(
            "pose_smoothing_alpha",
            0.25);

    max_smoothed_translation_step_m_ =
        this->declare_parameter<double>(
            "max_smoothed_translation_step_m",
            0.20);

    max_smoothed_yaw_step_deg_ =
        this->declare_parameter<double>(
            "max_smoothed_yaw_step_deg",
            8.0);

    reset_smoothing_if_raw_jump_m_ =
        this->declare_parameter<double>(
            "reset_smoothing_if_raw_jump_m",
            8.0);

    reset_smoothing_if_raw_jump_yaw_deg_ =
        this->declare_parameter<double>(
            "reset_smoothing_if_raw_jump_yaw_deg",
            120.0);

    reset_smoothing_if_no_publish_s_ =
        this->declare_parameter<double>(
            "reset_smoothing_if_no_publish_s",
            2.0);

    pose_smoothing_alpha_ =
        ClampDouble(
            pose_smoothing_alpha_,
            0.0,
            1.0);

    if (max_smoothed_translation_step_m_ <= 0.0)
        max_smoothed_translation_step_m_ = 0.20;

    if (max_smoothed_yaw_step_deg_ <= 0.0)
        max_smoothed_yaw_step_deg_ = 8.0;

    if (reset_smoothing_if_raw_jump_m_ <= 0.0)
        reset_smoothing_if_raw_jump_m_ = 8.0;

    if (reset_smoothing_if_raw_jump_yaw_deg_ <= 0.0)
        reset_smoothing_if_raw_jump_yaw_deg_ = 120.0;

    if (reset_smoothing_if_no_publish_s_ <= 0.0)
        reset_smoothing_if_no_publish_s_ = 2.0;

    last_smoothed_publish_time_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("orbslam/pose_local", 10, std::bind(&GlobalPoseCorrector::LocalPoseCallback, this, std::placeholders::_1));

    map_correction_sub_ = this->create_subscription<orbslam3_msgs::msg::MapCorrection>("map_correction", 10, std::bind(&GlobalPoseCorrector::MapCorrectionCallback, this, std::placeholders::_1));

    corrected_keyframes_sub_ = this->create_subscription<orbslam3_msgs::msg::CorrectedKeyFrameArray>("corrected_keyframes", 10, std::bind(&GlobalPoseCorrector::CorrectedKeyFramesCallback, this, std::placeholders::_1));

    global_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_global_corrected", 10);

    global_pose_raw_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose_global_corrected_raw",
            10);

    global_body_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose_global_body_corrected",
            10);

    global_body_pose_raw_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose_global_body_corrected_raw",
            10);

    RCLCPP_WARN(
        this->get_logger(),
        "[POSE4A-CONFIG] drone_%u world_frame=%s use_corrected_keyframes=%s "
        "max_nearest_kf_dist=%.3f map_stale_warn_age=%.3f "
        "map_hard_timeout=%.3f max_corrected_kfs_age=%.3f "
        "publish_using_last_correction_when_stale=%s "
        "fallback_if_corrected_kfs_stale=%s warn_jump_m=%.3f warn_jump_yaw_deg=%.3f "
        "smoothing=%s main_topic_smoothed=%s alpha=%.3f max_step_m=%.3f "
        "max_yaw_step_deg=%.3f reset_jump_m=%.3f reset_jump_yaw_deg=%.3f "
        "reset_no_publish_s=%.3f",
        drone_id_,
        world_frame_.c_str(),
        use_corrected_keyframes_ ? "true" : "false",
        max_nearest_kf_distance_m_,
        map_correction_stale_warn_age_s_,
        map_correction_hard_timeout_s_,
        max_corrected_keyframes_age_s_,
        publish_using_last_correction_when_stale_ ? "true" : "false",
        allow_rigid_fallback_when_corrected_kfs_stale_ ? "true" : "false",
        warn_output_jump_translation_m_,
        warn_output_jump_yaw_deg_,
        enable_pose_smoothing_ ? "true" : "false",
        publish_smoothed_pose_on_main_topic_ ? "true" : "false",
        pose_smoothing_alpha_,
        max_smoothed_translation_step_m_,
        max_smoothed_yaw_step_deg_,
        reset_smoothing_if_raw_jump_m_,
        reset_smoothing_if_raw_jump_yaw_deg_,
        reset_smoothing_if_no_publish_s_);

    publish_body_pose_ =
        this->declare_parameter<bool>(
            "publish_body_pose",
            true);

    body_T_camera_x_ =
        this->declare_parameter<double>(
            "body_T_camera_x",
            0.10);

    body_T_camera_y_ =
        this->declare_parameter<double>(
            "body_T_camera_y",
            0.03);

    body_T_camera_z_ =
        this->declare_parameter<double>(
            "body_T_camera_z",
            0.03);

    body_T_camera_roll_deg_ =
        this->declare_parameter<double>(
            "body_T_camera_roll_deg",
            0.0);

    body_T_camera_pitch_deg_ =
        this->declare_parameter<double>(
            "body_T_camera_pitch_deg",
            0.0);

    body_T_camera_yaw_deg_ =
        this->declare_parameter<double>(
            "body_T_camera_yaw_deg",
            0.0);

    use_camera_optical_frame_convention_ =
        this->declare_parameter<bool>(
            "use_camera_optical_frame_convention",
            true);

    const double deg_to_rad =
        M_PI / 180.0;

    tf2::Quaternion q_body_camera =
        BuildBodyTCameraQuaternion();

    q_body_camera.normalize();

    body_T_camera_.setOrigin(
        tf2::Vector3(
            body_T_camera_x_,
            body_T_camera_y_,
            body_T_camera_z_));

    body_T_camera_.setRotation(
        q_body_camera);

    camera_T_body_ =
        body_T_camera_.inverse();

    enable_gt_pose_diagnostics_ =
        this->declare_parameter<bool>(
            "enable_gt_pose_diagnostics",
            true);

    warn_body_position_error_m_ =
        this->declare_parameter<double>(
            "warn_body_position_error_m",
            0.75);

    warn_body_yaw_error_deg_ =
        this->declare_parameter<double>(
            "warn_body_yaw_error_deg",
            20.0);

    if (enable_gt_pose_diagnostics_)
    {
        gt_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "sensor/GT/pose",
                20,
                std::bind(
                    &GlobalPoseCorrector::GroundTruthPoseCallback,
                    this,
                    std::placeholders::_1));
    }

    RCLCPP_WARN(
        this->get_logger(),
        "[CALIB0-BODY-EXTRINSIC] drone_%u publish_body_pose=%s "
        "use_optical_convention=%s "
        "body_T_camera_xyz=(%.3f, %.3f, %.3f) "
        "body_T_camera_rpy_deg=(%.3f, %.3f, %.3f) "
        "body_T_camera_quat=(%.4f, %.4f, %.4f, %.4f) "
        "gt_diag=%s warn_body_err_m=%.3f warn_body_yaw_deg=%.3f",
        drone_id_,
        publish_body_pose_ ? "true" : "false",
        use_camera_optical_frame_convention_ ? "true" : "false",
        body_T_camera_x_,
        body_T_camera_y_,
        body_T_camera_z_,
        body_T_camera_roll_deg_,
        body_T_camera_pitch_deg_,
        body_T_camera_yaw_deg_,
        q_body_camera.x(),
        q_body_camera.y(),
        q_body_camera.z(),
        q_body_camera.w(),
        enable_gt_pose_diagnostics_ ? "true" : "false",
        warn_body_position_error_m_,
        warn_body_yaw_error_deg_);

    RCLCPP_WARN(
        this->get_logger(),
        "[CONFIG-CORRECTOR] drone_%u world_frame=%s "
        "use_corrected_keyframes=%s max_nearest_kf_distance_m=%.3f "
        "map_stale_warn_age_s=%.3f map_hard_timeout_s=%.3f "
        "max_corrected_keyframes_age_s=%.3f "
        "smoothing=%s publish_smoothed_on_main=%s alpha=%.3f "
        "max_step_m=%.3f max_yaw_step_deg=%.3f "
        "publish_body_pose=%s body_T_camera_xyz=(%.3f, %.3f, %.3f) "
        "body_T_camera_rpy_deg=(%.3f, %.3f, %.3f) "
        "note=does_not_use_fused_multi_policy_params",
        drone_id_,
        world_frame_.c_str(),
        use_corrected_keyframes_ ? "true" : "false",
        max_nearest_kf_distance_m_,
        map_correction_stale_warn_age_s_,
        map_correction_hard_timeout_s_,
        max_corrected_keyframes_age_s_,
        enable_pose_smoothing_ ? "true" : "false",
        publish_smoothed_pose_on_main_topic_ ? "true" : "false",
        pose_smoothing_alpha_,
        max_smoothed_translation_step_m_,
        max_smoothed_yaw_step_deg_,
        publish_body_pose_ ? "true" : "false",
        body_T_camera_x_,
        body_T_camera_y_,
        body_T_camera_z_,
        body_T_camera_roll_deg_,
        body_T_camera_pitch_deg_,
        body_T_camera_yaw_deg_);
}


/* CALLBACKS */

void GlobalPoseCorrector::MapCorrectionCallback(
    const orbslam3_msgs::msg::MapCorrection::SharedPtr msg)
{
    if (msg->drone_id != drone_id_)
        return;

    tf2::Transform T_world_local =
        TransformMsgToTf2(msg->world_t_local_map);

    if (!IsFiniteTransform(T_world_local))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[POSE4A-CORRECTION-RX-REJECT] drone_%u reason=non_finite_world_T_local "
            "seq=%lu world_frame=%s local_frame=%s",
            drone_id_,
            msg->map_sequence,
            msg->world_frame.c_str(),
            msg->local_map_frame.c_str());

        return;
    }

    world_T_local_map_ =
        msg->world_t_local_map;

    has_world_T_local_map_ =
        true;

    last_map_correction_rx_time_ =
        now();

    RCLCPP_INFO(
        this->get_logger(),
        "[POSE4B-CORRECTION-UPDATE] drone_%u received_new_world_T_local "
        "smoothing_initialized=%s",
        drone_id_,
        has_smoothed_pose_ ? "true" : "false");

    if (!msg->world_frame.empty())
        world_frame_ = msg->world_frame;

    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "[POSE4A-CORRECTION-RX] drone_%u seq=%lu world_frame=%s local_frame=%s "
        "translation=(%.3f, %.3f, %.3f) rotation=(%.4f, %.4f, %.4f, %.4f)",
        drone_id_,
        msg->map_sequence,
        msg->world_frame.c_str(),
        msg->local_map_frame.c_str(),
        msg->world_t_local_map.translation.x,
        msg->world_t_local_map.translation.y,
        msg->world_t_local_map.translation.z,
        msg->world_t_local_map.rotation.x,
        msg->world_t_local_map.rotation.y,
        msg->world_t_local_map.rotation.z,
        msg->world_t_local_map.rotation.w);
}


void GlobalPoseCorrector::CorrectedKeyFramesCallback(
    const orbslam3_msgs::msg::CorrectedKeyFrameArray::SharedPtr msg)
{
    if (msg->drone_id != drone_id_)
        return;

    corrected_keyframes_.clear();

    size_t rejected_non_finite = 0;

    for (const auto& kf_msg : msg->keyframes)
    {
        if (!IsFinitePose(kf_msg.original_local_pose) ||
            !IsFinitePose(kf_msg.original_global_pose) ||
            !IsFinitePose(kf_msg.corrected_global_pose))
        {
            rejected_non_finite++;
            continue;
        }

        CorrectedKeyFrameData data;

        data.keyframe_id =
            kf_msg.keyframe_id;

        data.original_local_pose =
            kf_msg.original_local_pose;

        data.original_global_pose =
            kf_msg.original_global_pose;

        data.corrected_global_pose =
            kf_msg.corrected_global_pose;

        corrected_keyframes_[data.keyframe_id] =
            data;
    }

    has_corrected_keyframes_ =
        !corrected_keyframes_.empty();

    if (has_corrected_keyframes_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[POSE4B-CORRECTED-KFS-ACTIVE] drone_%u corrected_kfs=%zu "
            "will_try_CORRECTED_KEYFRAME_RELATIVE=true",
            drone_id_,
            corrected_keyframes_.size());
    }

    last_corrected_keyframes_rx_time_ =
        now();

    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "[POSE4A-CORRECTED-KFS-RX] drone_%u corrected_kfs=%zu rejected_non_finite=%zu "
        "use_corrected_keyframes=%s world_frame=%s local_frame=%s",
        drone_id_,
        corrected_keyframes_.size(),
        rejected_non_finite,
        use_corrected_keyframes_ ? "true" : "false",
        msg->world_frame.c_str(),
        msg->local_map_frame.c_str());
}


void GlobalPoseCorrector::LocalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_local_pose_rx_time_ =
        now();

    PoseCorrectionDiagnostics diag;

    if (!has_world_T_local_map_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-NO-PUBLISH] drone_%u mode=NO_CORRECTION reason=no_world_T_local_map "
            "local_stamp=%.6f",
            drone_id_,
            rclcpp::Time(msg->header.stamp).seconds());

        return;
    }

    tf2::Transform T_local_camera =
        PoseMsgToTf2(msg->pose);

    if (!IsFiniteTransform(T_local_camera))
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-NO-PUBLISH] drone_%u mode=INVALID_TRANSFORM reason=local_pose_not_finite",
            drone_id_);

        return;
    }

    tf2::Transform T_world_local =
        TransformMsgToTf2(world_T_local_map_);

    if (!IsFiniteTransform(T_world_local))
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-NO-PUBLISH] drone_%u mode=INVALID_TRANSFORM reason=world_T_local_not_finite",
            drone_id_);

        return;
    }

    diag.map_correction_age_s =
        AgeSeconds(last_map_correction_rx_time_);

    const bool map_correction_received =
        std::isfinite(diag.map_correction_age_s);

    const bool map_correction_warning_stale =
        map_correction_received &&
        diag.map_correction_age_s > map_correction_stale_warn_age_s_;

    const bool map_correction_hard_timeout =
        !map_correction_received ||
        diag.map_correction_age_s > map_correction_hard_timeout_s_;

    diag.has_valid_map_correction =
        !map_correction_hard_timeout;

    if (map_correction_hard_timeout)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "[POSE4A-NO-PUBLISH] drone_%u mode=MAP_CORRECTION_TIMEOUT "
            "map_correction_age=%.3f hard_timeout=%.3f action=skip_publish",
            drone_id_,
            diag.map_correction_age_s,
            map_correction_hard_timeout_s_);

        return;
    }

    if (map_correction_warning_stale)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-STALE-WARN] drone_%u map_correction_age=%.3f warn_age=%.3f "
            "hard_timeout=%.3f action=continue_using_last_valid_correction",
            drone_id_,
            diag.map_correction_age_s,
            map_correction_stale_warn_age_s_,
            map_correction_hard_timeout_s_);
    }

    tf2::Transform T_world_camera;
    bool used_corrected_keyframe = false;

    const bool map_correction_warning_stale_for_mode =
        diag.map_correction_age_s > map_correction_stale_warn_age_s_;

    diag.mode =
        map_correction_warning_stale_for_mode
            ? "RIGID_SUBMAP_CORRECTION_STALE_MAP_WARNING"
            : "RIGID_SUBMAP_CORRECTION";

    diag.corrected_keyframes_age_s =
        AgeSeconds(last_corrected_keyframes_rx_time_);

    const bool corrected_kfs_fresh =
        has_corrected_keyframes_ &&
        std::isfinite(diag.corrected_keyframes_age_s) &&
        diag.corrected_keyframes_age_s <= max_corrected_keyframes_age_s_;

    diag.has_valid_corrected_keyframes =
        corrected_kfs_fresh;

    if (use_corrected_keyframes_ &&
        has_corrected_keyframes_ &&
        !corrected_kfs_fresh)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-CORRECTED-KFS-STALE] drone_%u age=%.3f max_age=%.3f "
            "fallback_allowed=%s",
            drone_id_,
            diag.corrected_keyframes_age_s,
            max_corrected_keyframes_age_s_,
            allow_rigid_fallback_when_corrected_kfs_stale_ ? "true" : "false");
    }

    if (use_corrected_keyframes_ &&
        corrected_kfs_fresh)
    {
        double best_dist2 =
            std::numeric_limits<double>::max();

        const CorrectedKeyFrameData* best_kf =
            nullptr;

        tf2::Vector3 current_pos =
            T_local_camera.getOrigin();

        for (const auto& [kf_id, kf_data] : corrected_keyframes_)
        {
            (void)kf_id;

            tf2::Transform T_local_kf =
                PoseMsgToTf2(kf_data.original_local_pose);

            if (!IsFiniteTransform(T_local_kf))
                continue;

            tf2::Vector3 kf_pos =
                T_local_kf.getOrigin();

            const double dx =
                current_pos.x() - kf_pos.x();

            const double dy =
                current_pos.y() - kf_pos.y();

            const double dz =
                current_pos.z() - kf_pos.z();

            const double dist2 =
                dx * dx + dy * dy + dz * dz;

            if (dist2 < best_dist2)
            {
                best_dist2 =
                    dist2;

                best_kf =
                    &kf_data;
            }
        }

        const double max_dist2 =
            max_nearest_kf_distance_m_ *
            max_nearest_kf_distance_m_;

        if (best_kf)
        {
            diag.nearest_kf_distance_m =
                std::sqrt(best_dist2);

            diag.selected_keyframe_id =
                best_kf->keyframe_id;

            diag.selected_kf_distance_m =
                diag.nearest_kf_distance_m;
        }

        if (best_kf &&
            best_dist2 <= max_dist2)
        {
            tf2::Transform T_local_kf =
                PoseMsgToTf2(best_kf->original_local_pose);

            tf2::Transform T_world_kf_corrected =
                PoseMsgToTf2(best_kf->corrected_global_pose);

            if (IsFiniteTransform(T_local_kf) &&
                IsFiniteTransform(T_world_kf_corrected))
            {
                // T_world_current =
                //   T_world_kf_corrected *
                //   inv(T_local_kf) *
                //   T_local_current
                tf2::Transform C_kf =
                    T_world_kf_corrected *
                    T_local_kf.inverse();

                T_world_camera =
                    C_kf *
                    T_local_camera;

                used_corrected_keyframe =
                    true;

                diag.mode =
                    "CORRECTED_KEYFRAME_RELATIVE";

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "[POSE4B-KF-RELATIVE-ACTIVE] drone_%u selected_kf=%lu dist=%.3f "
                    "using_corrected_keyframe_pose=true",
                    drone_id_,
                    diag.selected_keyframe_id,
                    diag.selected_kf_distance_m);
            }
        }

        if (best_kf &&
            !used_corrected_keyframe)
        {
            diag.mode =
                "CORRECTED_KEYFRAME_TOO_FAR_FALLBACK_RIGID";
        }
    }
    else if (!use_corrected_keyframes_)
    {
        diag.mode =
            map_correction_warning_stale_for_mode
                ? "RIGID_SUBMAP_CORRECTION_STALE_MAP_WARNING_DISABLED_CORRECTED_KFS"
                : "RIGID_SUBMAP_CORRECTION_DISABLED_CORRECTED_KFS";
    }
    else if (!has_corrected_keyframes_)
    {
        diag.mode =
            map_correction_warning_stale_for_mode
                ? "RIGID_SUBMAP_CORRECTION_STALE_MAP_WARNING_NO_CORRECTED_KFS"
                : "RIGID_SUBMAP_CORRECTION_NO_CORRECTED_KFS";
    }
    else if (!corrected_kfs_fresh)
    {
        if (!allow_rigid_fallback_when_corrected_kfs_stale_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "[POSE4A-NO-PUBLISH] drone_%u mode=STALE_CORRECTED_KEYFRAMES "
                "age=%.3f max_age=%.3f action=skip_publish",
                drone_id_,
                diag.corrected_keyframes_age_s,
                max_corrected_keyframes_age_s_);

            return;
        }

        diag.mode =
            map_correction_warning_stale_for_mode
                ? "RIGID_SUBMAP_CORRECTION_STALE_MAP_WARNING_STALE_CORRECTED_KFS"
                : "STALE_CORRECTED_KEYFRAMES_FALLBACK_RIGID";
    }

    if (!used_corrected_keyframe)
    {
        T_world_camera =
            T_world_local *
            T_local_camera;
    }

    if (!IsFiniteTransform(T_world_camera))
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[POSE4A-NO-PUBLISH] drone_%u mode=INVALID_TRANSFORM reason=output_not_finite",
            drone_id_);

        return;
    }

    diag.output_is_finite =
        true;

    if (has_last_published_pose_)
    {
        diag.output_jump_m =
            TranslationDistance(
                last_published_world_T_camera_,
                T_world_camera);

        diag.output_jump_yaw_deg =
            YawDifferenceDeg(
                last_published_world_T_camera_,
                T_world_camera);

        if (diag.output_jump_m > warn_output_jump_translation_m_ ||
            diag.output_jump_yaw_deg > warn_output_jump_yaw_deg_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[POSE4A-JUMP-WARN] drone_%u mode=%s jump_m=%.3f max_m=%.3f "
                "jump_yaw_deg=%.3f max_yaw_deg=%.3f "
                "selected_kf=%lu nearest_kf_dist=%.3f map_age=%.3f kfs_age=%.3f",
                drone_id_,
                diag.mode.c_str(),
                diag.output_jump_m,
                warn_output_jump_translation_m_,
                diag.output_jump_yaw_deg,
                warn_output_jump_yaw_deg_,
                diag.selected_keyframe_id,
                diag.nearest_kf_distance_m,
                diag.map_correction_age_s,
                diag.corrected_keyframes_age_s);
        }
    }

    // ============================================================
    // FASE 4B:
    // Publicar raw y smoothed.
    // ============================================================

    geometry_msgs::msg::PoseStamped raw_out;

    raw_out.header.stamp =
        msg->header.stamp;

    raw_out.header.frame_id =
        world_frame_;

    raw_out.pose =
        Tf2ToPoseMsg(T_world_camera);

    global_pose_raw_pub_->publish(raw_out);

    tf2::Transform smoothed_T_world_camera =
        T_world_camera;

    bool smoothing_used = false;
    bool smoothing_reset = false;
    bool smoothing_translation_limited = false;
    bool smoothing_yaw_limited = false;

    double raw_to_previous_smoothed_m = 0.0;
    double raw_to_previous_smoothed_yaw_deg = 0.0;
    double raw_to_smoothed_m = 0.0;
    double raw_to_smoothed_yaw_deg = 0.0;

    const rclcpp::Time t_now =
        now();

    double seconds_since_last_smoothed_publish =
        std::numeric_limits<double>::infinity();

    if (last_smoothed_publish_time_.nanoseconds() != 0)
    {
        seconds_since_last_smoothed_publish =
            (t_now - last_smoothed_publish_time_).seconds();
    }

    if (!enable_pose_smoothing_)
    {
        smoothed_T_world_camera =
            T_world_camera;

        smoothing_reset =
            true;
    }
    else if (!has_smoothed_pose_)
    {
        smoothed_T_world_camera =
            T_world_camera;

        smoothing_reset =
            true;

        RCLCPP_WARN(
            this->get_logger(),
            "[POSE4B-SMOOTH-RESET] drone_%u reason=first_smoothed_pose",
            drone_id_);
    }
    else
    {
        raw_to_previous_smoothed_m =
            TranslationDistance(
                smoothed_world_T_camera_,
                T_world_camera);

        raw_to_previous_smoothed_yaw_deg =
            YawDifferenceDeg(
                smoothed_world_T_camera_,
                T_world_camera);

        const bool raw_jump_too_large =
            raw_to_previous_smoothed_m > reset_smoothing_if_raw_jump_m_ ||
            raw_to_previous_smoothed_yaw_deg > reset_smoothing_if_raw_jump_yaw_deg_;

        const bool no_publish_too_long =
            std::isfinite(seconds_since_last_smoothed_publish) &&
            seconds_since_last_smoothed_publish > reset_smoothing_if_no_publish_s_;

        if (raw_jump_too_large || no_publish_too_long)
        {
            smoothed_T_world_camera =
                T_world_camera;

            smoothing_reset =
                true;

            RCLCPP_WARN(
                this->get_logger(),
                "[POSE4B-SMOOTH-RESET] drone_%u reason=%s "
                "raw_jump_m=%.3f raw_jump_yaw_deg=%.3f "
                "seconds_since_last=%.3f",
                drone_id_,
                raw_jump_too_large
                    ? "raw_jump_too_large"
                    : "no_publish_too_long",
                raw_to_previous_smoothed_m,
                raw_to_previous_smoothed_yaw_deg,
                seconds_since_last_smoothed_publish);
        }
        else
        {
            smoothing_used =
                true;

            tf2::Vector3 from_t =
                smoothed_world_T_camera_.getOrigin();

            tf2::Vector3 to_t =
                T_world_camera.getOrigin();

            tf2::Vector3 delta_t =
                to_t - from_t;

            const double delta_norm =
                std::sqrt(
                    delta_t.x() * delta_t.x() +
                    delta_t.y() * delta_t.y() +
                    delta_t.z() * delta_t.z());

            double translation_alpha =
                pose_smoothing_alpha_;

            if (delta_norm > 1e-9)
            {
                const double max_alpha_by_step =
                    max_smoothed_translation_step_m_ /
                    delta_norm;

                if (translation_alpha > max_alpha_by_step)
                {
                    translation_alpha =
                        max_alpha_by_step;

                    smoothing_translation_limited =
                        true;
                }
            }

            translation_alpha =
                ClampDouble(
                    translation_alpha,
                    0.0,
                    1.0);

            tf2::Vector3 new_t =
                from_t +
                delta_t * translation_alpha;

            tf2::Quaternion from_q =
                smoothed_world_T_camera_.getRotation();

            tf2::Quaternion to_q =
                T_world_camera.getRotation();

            from_q =
                NormalizeQuaternion(from_q);

            to_q =
                NormalizeQuaternion(to_q);

            const double yaw_delta_deg =
                YawDifferenceDeg(
                    smoothed_world_T_camera_,
                    T_world_camera);

            double rotation_alpha =
                pose_smoothing_alpha_;

            if (yaw_delta_deg > 1e-9)
            {
                const double max_alpha_by_yaw =
                    max_smoothed_yaw_step_deg_ /
                    yaw_delta_deg;

                if (rotation_alpha > max_alpha_by_yaw)
                {
                    rotation_alpha =
                        max_alpha_by_yaw;

                    smoothing_yaw_limited =
                        true;
                }
            }

            rotation_alpha =
                ClampDouble(
                    rotation_alpha,
                    0.0,
                    1.0);

            tf2::Quaternion new_q =
                from_q.slerp(
                    to_q,
                    rotation_alpha);

            new_q =
                NormalizeQuaternion(new_q);

            smoothed_T_world_camera.setOrigin(new_t);
            smoothed_T_world_camera.setRotation(new_q);
        }
    }

    raw_to_smoothed_m =
        TranslationDistance(
            smoothed_T_world_camera,
            T_world_camera);

    raw_to_smoothed_yaw_deg =
        YawDifferenceDeg(
            smoothed_T_world_camera,
            T_world_camera);

    geometry_msgs::msg::PoseStamped smoothed_out;

    smoothed_out.header.stamp =
        msg->header.stamp;

    smoothed_out.header.frame_id =
        world_frame_;

    smoothed_out.pose =
        Tf2ToPoseMsg(smoothed_T_world_camera);

    if (publish_smoothed_pose_on_main_topic_)
    {
        global_pose_pub_->publish(smoothed_out);
    }
    else
    {
        global_pose_pub_->publish(raw_out);
    }

    smoothed_world_T_camera_ =
        smoothed_T_world_camera;

    has_smoothed_pose_ =
        true;

    last_smoothed_publish_time_ =
        t_now;

    last_published_world_T_camera_ =
        T_world_camera;

    has_last_published_pose_ =
        true;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "[POSE4B-PUBLISH] drone_%u mode=%s main=%s selected_kf=%lu "
        "nearest_kf_dist=%.3f selected_kf_dist=%.3f max_allowed=%.3f "
        "map_age=%.3f map_warn_age=%.3f map_hard_timeout=%.3f "
        "corrected_kfs_age=%.3f corrected_kfs_age_max=%.3f "
        "raw_jump_m=%.3f raw_jump_yaw_deg=%.3f "
        "raw_to_smoothed_m=%.3f raw_to_smoothed_yaw_deg=%.3f "
        "smoothing_used=%s smoothing_reset=%s limited_t=%s limited_yaw=%s "
        "output_finite=%s raw_pos=(%.3f, %.3f, %.3f) smoothed_pos=(%.3f, %.3f, %.3f)",
        drone_id_,
        diag.mode.c_str(),
        publish_smoothed_pose_on_main_topic_ ? "smoothed" : "raw",
        diag.selected_keyframe_id,
        diag.nearest_kf_distance_m,
        diag.selected_kf_distance_m,
        max_nearest_kf_distance_m_,
        diag.map_correction_age_s,
        map_correction_stale_warn_age_s_,
        map_correction_hard_timeout_s_,
        diag.corrected_keyframes_age_s,
        max_corrected_keyframes_age_s_,
        diag.output_jump_m,
        diag.output_jump_yaw_deg,
        raw_to_smoothed_m,
        raw_to_smoothed_yaw_deg,
        smoothing_used ? "true" : "false",
        smoothing_reset ? "true" : "false",
        smoothing_translation_limited ? "true" : "false",
        smoothing_yaw_limited ? "true" : "false",
        diag.output_is_finite ? "true" : "false",
        raw_out.pose.position.x,
        raw_out.pose.position.y,
        raw_out.pose.position.z,
        smoothed_out.pose.position.x,
        smoothed_out.pose.position.y,
        smoothed_out.pose.position.z);

    if (publish_body_pose_)
    {
        tf2::Transform raw_T_world_body =
            T_world_camera *
            camera_T_body_;

        tf2::Transform smoothed_T_world_body =
            smoothed_T_world_camera *
            camera_T_body_;

        if (IsFiniteTransform(raw_T_world_body) &&
            IsFiniteTransform(smoothed_T_world_body))
        {
            geometry_msgs::msg::PoseStamped raw_body_out;

            raw_body_out.header.stamp =
                msg->header.stamp;

            raw_body_out.header.frame_id =
                world_frame_;

            raw_body_out.pose =
                Tf2ToPoseMsg(raw_T_world_body);

            geometry_msgs::msg::PoseStamped smoothed_body_out;

            smoothed_body_out.header.stamp =
                msg->header.stamp;

            smoothed_body_out.header.frame_id =
                world_frame_;

            smoothed_body_out.pose =
                Tf2ToPoseMsg(smoothed_T_world_body);

            global_body_pose_raw_pub_->publish(raw_body_out);
            global_body_pose_pub_->publish(smoothed_body_out);

            if (enable_gt_pose_diagnostics_ &&
                has_gt_body_pose_)
            {
                tf2::Transform T_world_body_gt =
                    PoseMsgToTf2(last_gt_body_pose_.pose);

                tf2::Transform T_world_body_est =
                    PoseMsgToTf2(smoothed_body_out.pose);

                const double pos_err_m =
                    TranslationDistance(
                        T_world_body_gt,
                        T_world_body_est);

                const double yaw_err_deg =
                    YawDifferenceDeg(
                        T_world_body_gt,
                        T_world_body_est);

                const bool warn =
                    pos_err_m > warn_body_position_error_m_ ||
                    yaw_err_deg > warn_body_yaw_error_deg_;

                if (warn)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        500,
                        "[CALIB0-BODY-GT-ERROR] drone_%u mode=%s "
                        "pos_err=%.3f warn_pos=%.3f yaw_err_deg=%.3f warn_yaw=%.3f "
                        "est_pos=(%.3f, %.3f, %.3f) gt_pos=(%.3f, %.3f, %.3f) "
                        "selected_kf=%lu selected_kf_dist=%.3f map_age=%.3f kfs_age=%.3f",
                        drone_id_,
                        diag.mode.c_str(),
                        pos_err_m,
                        warn_body_position_error_m_,
                        yaw_err_deg,
                        warn_body_yaw_error_deg_,
                        smoothed_body_out.pose.position.x,
                        smoothed_body_out.pose.position.y,
                        smoothed_body_out.pose.position.z,
                        last_gt_body_pose_.pose.position.x,
                        last_gt_body_pose_.pose.position.y,
                        last_gt_body_pose_.pose.position.z,
                        diag.selected_keyframe_id,
                        diag.selected_kf_distance_m,
                        diag.map_correction_age_s,
                        diag.corrected_keyframes_age_s);
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "[POSE4C-BODY-GT-OK] drone_%u mode=%s "
                        "pos_err=%.3f yaw_err_deg=%.3f "
                        "selected_kf=%lu selected_kf_dist=%.3f",
                        drone_id_,
                        diag.mode.c_str(),
                        pos_err_m,
                        yaw_err_deg,
                        diag.selected_keyframe_id,
                        diag.selected_kf_distance_m);
                }
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "[POSE4C-BODY-PUBLISH] drone_%u mode=%s "
                "body_raw_pos=(%.3f, %.3f, %.3f) "
                "body_smoothed_pos=(%.3f, %.3f, %.3f) "
                "selected_kf=%lu selected_kf_dist=%.3f map_age=%.3f kfs_age=%.3f",
                drone_id_,
                diag.mode.c_str(),
                raw_body_out.pose.position.x,
                raw_body_out.pose.position.y,
                raw_body_out.pose.position.z,
                smoothed_body_out.pose.position.x,
                smoothed_body_out.pose.position.y,
                smoothed_body_out.pose.position.z,
                diag.selected_keyframe_id,
                diag.selected_kf_distance_m,
                diag.map_correction_age_s,
                diag.corrected_keyframes_age_s);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[POSE4C-BODY-NO-PUBLISH] drone_%u reason=non_finite_body_pose",
                drone_id_);
        }
    }
}





/* FUNCIONES AUXILIARES */

tf2::Transform GlobalPoseCorrector::PoseMsgToTf2(const geometry_msgs::msg::Pose& pose)
{
    tf2::Transform tf;

    tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));

    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    q.normalize();
    tf.setRotation(q);

    return tf;
}

tf2::Transform GlobalPoseCorrector::TransformMsgToTf2(const geometry_msgs::msg::Transform& transform)
{
    tf2::Transform tf;

    tf.setOrigin(tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));

    tf2::Quaternion q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);

    q.normalize();
    tf.setRotation(q);

    return tf;
}

geometry_msgs::msg::Pose GlobalPoseCorrector::Tf2ToPoseMsg(const tf2::Transform& tf)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.position.z = tf.getOrigin().z();

    tf2::Quaternion q = tf.getRotation();
    q.normalize();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}




bool GlobalPoseCorrector::IsFiniteTransform(
    const tf2::Transform& tf) const
{
    const tf2::Vector3 t =
        tf.getOrigin();

    const tf2::Quaternion q =
        tf.getRotation();

    return
        std::isfinite(t.x()) &&
        std::isfinite(t.y()) &&
        std::isfinite(t.z()) &&
        std::isfinite(q.x()) &&
        std::isfinite(q.y()) &&
        std::isfinite(q.z()) &&
        std::isfinite(q.w()) &&
        std::abs(q.length2()) > 1e-12;
}


bool GlobalPoseCorrector::IsFinitePose(
    const geometry_msgs::msg::Pose& pose) const
{
    return
        std::isfinite(pose.position.x) &&
        std::isfinite(pose.position.y) &&
        std::isfinite(pose.position.z) &&
        std::isfinite(pose.orientation.x) &&
        std::isfinite(pose.orientation.y) &&
        std::isfinite(pose.orientation.z) &&
        std::isfinite(pose.orientation.w) &&
        (
            pose.orientation.x * pose.orientation.x +
            pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z +
            pose.orientation.w * pose.orientation.w
        ) > 1e-12;
}


double GlobalPoseCorrector::TranslationDistance(
    const tf2::Transform& a,
    const tf2::Transform& b) const
{
    const tf2::Vector3 da =
        a.getOrigin() -
        b.getOrigin();

    return
        std::sqrt(
            da.x() * da.x() +
            da.y() * da.y() +
            da.z() * da.z());
}


double GlobalPoseCorrector::YawFromTransform(
    const tf2::Transform& tf) const
{
    const tf2::Matrix3x3 m(tf.getRotation());

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    m.getRPY(roll, pitch, yaw);

    return yaw;
}


double GlobalPoseCorrector::NormalizeAngleRad(
    double a) const
{
    while (a > M_PI)
        a -= 2.0 * M_PI;

    while (a < -M_PI)
        a += 2.0 * M_PI;

    return a;
}


double GlobalPoseCorrector::YawDifferenceDeg(
    const tf2::Transform& a,
    const tf2::Transform& b) const
{
    const double yaw_a =
        YawFromTransform(a);

    const double yaw_b =
        YawFromTransform(b);

    const double dyaw =
        NormalizeAngleRad(yaw_b - yaw_a);

    return
        std::abs(dyaw) *
        180.0 /
        M_PI;
}


double GlobalPoseCorrector::AgeSeconds(
    const rclcpp::Time& stamp) const
{
    if (stamp.nanoseconds() == 0)
        return std::numeric_limits<double>::infinity();

    return
        (now() - stamp).seconds();
}


double GlobalPoseCorrector::ClampDouble(
    double value,
    double lo,
    double hi) const
{
    if (value < lo)
        return lo;

    if (value > hi)
        return hi;

    return value;
}


tf2::Quaternion GlobalPoseCorrector::NormalizeQuaternion(
    const tf2::Quaternion& q) const
{
    tf2::Quaternion out =
        q;

    if (out.length2() < 1e-12 ||
        !std::isfinite(out.x()) ||
        !std::isfinite(out.y()) ||
        !std::isfinite(out.z()) ||
        !std::isfinite(out.w()))
    {
        out.setValue(0.0, 0.0, 0.0, 1.0);
        return out;
    }

    out.normalize();

    return out;
}


tf2::Transform GlobalPoseCorrector::InterpolateTransform(
    const tf2::Transform& from,
    const tf2::Transform& to,
    double alpha) const
{
    alpha =
        ClampDouble(
            alpha,
            0.0,
            1.0);

    tf2::Vector3 from_t =
        from.getOrigin();

    tf2::Vector3 to_t =
        to.getOrigin();

    tf2::Vector3 out_t =
        from_t +
        (to_t - from_t) * alpha;

    tf2::Quaternion from_q =
        NormalizeQuaternion(
            from.getRotation());

    tf2::Quaternion to_q =
        NormalizeQuaternion(
            to.getRotation());

    tf2::Quaternion out_q =
        from_q.slerp(
            to_q,
            alpha);

    out_q =
        NormalizeQuaternion(out_q);

    tf2::Transform out;
    out.setOrigin(out_t);
    out.setRotation(out_q);

    return out;
}


tf2::Quaternion GlobalPoseCorrector::BuildBodyTCameraQuaternion() const
{
    if (use_camera_optical_frame_convention_)
    {
        // ========================================================
        // Convención óptica típica:
        //
        // body:
        //   x = forward
        //   y = left
        //   z = up
        //
        // camera optical:
        //   z = forward/depth
        //   x = right
        //   y = down
        //
        // Por tanto:
        //   camera_x = -body_y
        //   camera_y = -body_z
        //   camera_z =  body_x
        //
        // R_body_camera tiene como columnas los ejes de la cámara
        // expresados en body:
        //
        //   col0 = camera_x in body = ( 0, -1,  0)
        //   col1 = camera_y in body = ( 0,  0, -1)
        //   col2 = camera_z in body = ( 1,  0,  0)
        //
        // Matriz:
        //   [ 0  0  1
        //    -1  0  0
        //     0 -1  0 ]
        //
        // Equivale aproximadamente a RPY = (-90, 0, -90).
        // ========================================================

        tf2::Matrix3x3 R_body_camera;

        R_body_camera.setValue(
            0.0,  0.0,  1.0,
           -1.0,  0.0,  0.0,
            0.0, -1.0,  0.0);

        tf2::Quaternion q_body_camera;

        R_body_camera.getRotation(
            q_body_camera);

        q_body_camera.normalize();

        return q_body_camera;
    }

    const double deg_to_rad =
        M_PI / 180.0;

    tf2::Quaternion q_body_camera;

    q_body_camera.setRPY(
        body_T_camera_roll_deg_ * deg_to_rad,
        body_T_camera_pitch_deg_ * deg_to_rad,
        body_T_camera_yaw_deg_ * deg_to_rad);

    q_body_camera.normalize();

    return q_body_camera;
}


void GlobalPoseCorrector::GroundTruthPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_gt_body_pose_ =
        *msg;

    has_gt_body_pose_ =
        true;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GlobalPoseCorrector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}