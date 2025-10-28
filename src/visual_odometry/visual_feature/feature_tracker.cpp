#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

void yoloDetect::using_once()
{
    std::call_once(flag_, [this](){this->init();});
}

void yoloDetect::init()
{
    try
    {
        detectioNet = cv::dnn::readNetFromONNX(model_path);
    }
    catch (const cv::Exception& e)
    {
        ROS_ERROR("读取模型失败: %s", e.what());
    }

}

dataImg yoloDetect::preprocess(cv::Mat &img)
{
    if (img.empty())
    {
        std::cout<<"lab is empty"<<std::endl;
        return dataImg();
    }

    cv::Mat bgr_img = img.clone();

    // 1.获取图像尺寸
    int h = bgr_img.rows;
    int w = bgr_img.cols;
    int th = target_size;
    int tw = target_size;

    // 2.计算缩放比例
    float scale = std::min(static_cast<float>(tw) / w, static_cast<float>(th) / h);
    scale = std::max(scale, 0.01f);
    int new_w = static_cast<int>(w * scale);
    int new_h = static_cast<int>(h * scale);

    // 3.缩放图像
    cv::Mat resized_img;
    resize(bgr_img, resized_img, cv::Size(new_w, new_h));

    // 4.计算填充量
    int padW = tw - new_w;
    int padH = th - new_h;

    int left = padW / 2;
    int right = padW - left;
    int top = padH / 2;
    int bottom = padH - top;

    // 5.填充图像
    cv::Mat padded_img;
    copyMakeBorder(resized_img, padded_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 6.创建blob
    cv::Mat blob = cv::dnn::blobFromImage(padded_img, 1.0/255.0, cv::Size(target_size, target_size), cv::Scalar(0,0,0), true, CV_32F);

    dataImg imgdata;
    imgdata.scale = scale;
    imgdata.padW = padW;
    imgdata.padH = padH;
    imgdata.input = bgr_img.clone();
    imgdata.blob = blob;

    return imgdata;
}

void yoloDetect::draw(cv::Mat& img, Target& target)
{
    rectangle(img, target.box, cv::Scalar(0, 255, 0), 2);
    std::string label = COCO_CLASSES[target.label];
    putText(img, label, cv::Point(target.box.x, target.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

void yoloDetect::Inference(cv::Mat &img)
{
    if (detectioNet.empty())
    {
        std::cout << "模型加载失败" << std::endl;
        return;
    }

    dataImg imgdata = preprocess(img);
    if (imgdata.blob.empty()) {
        std::cout << "预处理失败" << std::endl;
        return;
    }

    detectioNet.setInput(imgdata.blob);
    std::vector<cv::Mat> outputs;
    std::vector<std::string> outNames = detectioNet.getUnconnectedOutLayersNames();
    detectioNet.forward(outputs, outNames);

    if (outputs.empty()) {
        std::cout << "模型输出为空" << std::endl;
        return;
    }

    cv::Mat output = outputs[0];

    // 根据格式 float32[batch,84,anchors] 解析
    const int batch_size = output.size[0];  // 通常是 1
    const int num_features = output.size[1]; // 84 = 4(bbox) + 80(classes)
    const int num_anchors = output.size[2];  // anchors 数量

    std::cout << "Output shape: [" << batch_size << ", " << num_features << ", " << num_anchors << "]" << std::endl;

    // 重塑为更方便处理的格式: (84, num_anchors)
    cv::Mat output_reshaped = output.reshape(1, num_features);

    PreTargets preTargets;

    for (int i = 0; i < num_anchors; i++)
    {
        // YOLOv8 没有单独的 objectness 分数，直接使用类别分数
        cv::Mat scores = output_reshaped.rowRange(4, num_features).col(i);

        cv::Point class_id_point;
        double max_score;
        cv::minMaxLoc(scores, 0, &max_score, 0, &class_id_point);

        if (max_score > confidence_threshold_)
        {
            int class_id = class_id_point.y;

            // 获取边界框信息 (cx, cy, w, h) - 相对于网络输入尺寸 (640x640)
            float cx = output_reshaped.at<float>(0, i);
            float cy = output_reshaped.at<float>(1, i);
            float w = output_reshaped.at<float>(2, i);
            float h = output_reshaped.at<float>(3, i);

            // 坐标映射回原始图像
            // 注意：预处理时进行了缩放和填充，需要逆向操作
            float cx_unpad = (cx - imgdata.padW / 2) / imgdata.scale;
            float cy_unpad = (cy - imgdata.padH / 2) / imgdata.scale;
            float w_unpad = w / imgdata.scale;
            float h_unpad = h / imgdata.scale;

            // 计算边界框的左上角坐标和宽高
            int lx = static_cast<int>(cx_unpad - w_unpad / 2);
            int ly = static_cast<int>(cy_unpad - h_unpad / 2);
            int width = static_cast<int>(w_unpad);
            int height = static_cast<int>(h_unpad);

            // 边界检查
            lx = std::max(0, lx);
            ly = std::max(0, ly);
            width = std::min(width, img.cols - lx);
            height = std::min(height, img.rows - ly);

            if (width <= 5 || height <= 5) {
                continue;
            }

            cv::Rect box(lx, ly, width, height);
            preTargets.boxes.push_back(box);
            preTargets.confidences.push_back(static_cast<float>(max_score));
            preTargets.labels.push_back(class_id);
        }
    }

    // NMS 处理
    std::vector<int> indices;
    if (!preTargets.boxes.empty()) {
        // 按类别分组进行 NMS
        std::map<int, std::vector<int>> class_to_indices;
        for (size_t i = 0; i < preTargets.boxes.size(); i++) {
            class_to_indices[preTargets.labels[i]].push_back(i);
        }

        for (auto &pair : class_to_indices) {
            std::vector<cv::Rect> class_boxes;
            std::vector<float> class_confidences;
            std::vector<int> class_indices;

            for (int idx : pair.second) {
                class_boxes.push_back(preTargets.boxes[idx]);
                class_confidences.push_back(preTargets.confidences[idx]);
                class_indices.push_back(idx);
            }

            std::vector<int> class_nms_indices;
            cv::dnn::NMSBoxes(class_boxes, class_confidences,
                             confidence_threshold_, nms_threshold_, class_nms_indices);

            for (int nms_idx : class_nms_indices) {
                indices.push_back(class_indices[nms_idx]);
            }
        }

        // 绘制检测结果
        for (auto idx : indices) {
            Target target;
            target.box = preTargets.boxes[idx];
            target.confidence = preTargets.confidences[idx];
            target.label = preTargets.labels[idx];
            draw(img, target);
        }

        std::cout << "检测到 " << indices.size() << " 个目标" << std::endl;
    }
}


