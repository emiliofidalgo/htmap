#ifndef IMAGEHAMAP_H
#define IMAGEHAMAP_H

#include <opencv2/opencv.hpp>

namespace hamap
{

struct Image
{
    unsigned image_id;
    std::string image_filename;
    cv::Mat image;
    std::vector<cv::KeyPoint> kps;
    cv::Mat dscs;
    cv::Mat gdsc;

    void save(const std::string& file)
    {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        fs << "image_id" << static_cast<int>(image_id);
        fs << "image_filename" << image_filename;
        cv::write(fs, "kps", kps);
        fs << "dscs" << dscs;
        fs << "gdsc" << gdsc;
        fs.release();
    }

    void load(const std::string& file)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        image_id = static_cast<unsigned>((int)fs["image_id"]);        
        image_filename = (std::string)fs["image_filename"];        
        image = cv::imread(image_filename);        
        kps.clear();
        cv::read(fs["kps"], kps);        
        fs["dscs"] >> dscs;
        fs["gdsc"] >> gdsc;        
        fs.release();
    }

    static void loadGlobalDesc(const std::string& file, cv::Mat& desc)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        fs["gdsc"] >> desc;
        fs.release();
    }

    static void loadKeypointDesc(const std::string& file, std::vector<cv::KeyPoint>& kps, cv::Mat& desc)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        kps.clear();
        cv::read(fs["kps"], kps);
        fs["dscs"] >> desc;
        fs.release();
    }

    static void loadAllDesc(const std::string& file, cv::Mat& gdesc, std::vector<cv::KeyPoint>& kps, cv::Mat& desc)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        fs["gdsc"] >> gdesc;
        kps.clear();
        cv::read(fs["kps"], kps);
        fs["dscs"] >> desc;
        fs.release();
    }

    static void loadImage(const std::string& file, cv::Mat& img)
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        std::string image_filename = (std::string)fs["image_filename"];
        img = cv::imread(image_filename);
        fs.release();
    }
};

}

#endif // IMAGEHAMAP_H
