/*
* This file is part of htmap.
*
* Copyright (C) 2018 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
*
* htmap is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* htmap is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with htmap. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMAGEHAMAP_H
#define IMAGEHAMAP_H

#include <opencv2/opencv.hpp>

namespace htmap
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
