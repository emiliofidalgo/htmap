#include "hamap/imgdesc/GlobalDescriptor.h"

namespace hamap
{

void WISIFTDescriptor::describe(const cv::Mat& image, cv::Mat& desc)
{
    int width  = image.cols;
    int midpoint = static_cast<int>(width / 2);

    // Obtaining left and right patches.
    cv::Mat lmat = image.colRange(0, midpoint);
    cv::Mat rmat = image.colRange(midpoint, width);
    cv::Mat lpatch, rpatch;
    cv::resize(lmat, lpatch, cv::Size(128, 128));
    cv::resize(rmat, rpatch, cv::Size(128, 128));

    // Describing patches using SURF
    cv::KeyPoint kp;
    kp.pt.x = lpatch.cols / 2;
    kp.pt.y = lpatch.rows / 2;
    kp.size = 25;
    kp.response = 20000.0;
    kp.angle = 200.0;
    std::vector<cv::KeyPoint> kps;
    kps.push_back(kp);

    cv::Mat descp_l, descp_r;
    cv::SiftDescriptorExtractor extractor;
    extractor.compute(lpatch, kps, descp_l);
    extractor.compute(rpatch, kps, descp_r);

    // Merging the descriptors
    int ncomps = extractor.descriptorSize();
    desc = cv::Mat::zeros(1, ncomps * 2, CV_32F);
    descp_l.copyTo(desc.colRange(0, ncomps));
    descp_r.copyTo(desc.colRange(ncomps, ncomps * 2));
}

void WISURFDescriptor::describe(const cv::Mat& image, cv::Mat& desc)
{
    int width  = image.cols;
    int midpoint = static_cast<int>(width / 2);

    // Obtaining left and right patches.
    cv::Mat lmat = image.colRange(0, midpoint);
    cv::Mat rmat = image.colRange(midpoint, width);
    cv::Mat lpatch, rpatch;
    cv::resize(lmat, lpatch, cv::Size(128, 128));
    cv::resize(rmat, rpatch, cv::Size(128, 128));

    // Describing patches using SURF
    cv::KeyPoint kp;
    kp.pt.x = lpatch.cols / 2;
    kp.pt.y = lpatch.rows / 2;
    kp.size = 25;
    kp.response = 20000.0;
    kp.angle = 200.0;
    std::vector<cv::KeyPoint> kps;
    kps.push_back(kp);

    cv::Mat descp_l, descp_r;
    cv::SurfDescriptorExtractor extractor(5000.0, 4, 3, true, true);
    extractor.compute(lpatch, kps, descp_l);
    extractor.compute(rpatch, kps, descp_r);

    // Merging the descriptors
    int ncomps = extractor.descriptorSize();
    desc = cv::Mat::zeros(1, ncomps * 2, CV_32F);
    descp_l.copyTo(desc.colRange(0, ncomps));
    descp_r.copyTo(desc.colRange(ncomps, ncomps * 2));
}

void BRIEFGistDescriptor::describe(const cv::Mat& image, cv::Mat& desc)
{
    int width  = image.cols;
    int midpoint = static_cast<int>(width / 2);
    int desc_size = int(_desc_size / 2);

    // Obtaining left and right patches.
    cv::Mat lmat = image.colRange(0, midpoint);
    cv::Mat rmat = image.colRange(midpoint, width);
    cv::Mat lpatch, rpatch;
    cv::resize(lmat, lpatch, cv::Size(60, 60));
    cv::resize(rmat, rpatch, cv::Size(60, 60));

    // Defining patches using BRIEF
    cv::KeyPoint kp;
    kp.pt.x = lpatch.cols / 2;
    kp.pt.y = lpatch.rows / 2;
    kp.size = 1;
    std::vector<cv::KeyPoint> kps;
    kps.push_back(kp);

    // Describing patches using BRIEF
    cv::Mat descp_l, descp_r;
    cv::BriefDescriptorExtractor extractor(desc_size);
    extractor.compute(lpatch, kps, descp_l);
    extractor.compute(rpatch, kps, descp_r);

    // Merging the descriptors
    desc = cv::Mat::zeros(1, desc_size * 2, CV_8U);
    descp_l.copyTo(desc.colRange(0, desc_size));
    descp_r.copyTo(desc.colRange(desc_size, desc_size * 2));
}

void WILDBDescriptor::describe(const cv::Mat& image, cv::Mat& desc)
{
    int width  = image.cols;
    int midpoint = static_cast<int>(width / 2);
    int desc_size = int(_desc_size / 2);

    // Obtaining left and right patches.
    cv::Mat lmat = image.colRange(0, midpoint);
    cv::Mat rmat = image.colRange(midpoint, width);
    cv::Mat lpatch, rpatch;
    cv::resize(lmat, lpatch, cv::Size(60, 60));
    cv::resize(rmat, rpatch, cv::Size(60, 60));

    // Defining patches using LDB
    cv::KeyPoint kp;
    kp.pt.x = lpatch.cols / 2;
    kp.pt.y = lpatch.rows / 2;
    kp.size = 1;
    std::vector<cv::KeyPoint> kps;
    kps.push_back(kp);

    // Describing patches using BRIEF
    cv::Mat descp_l, descp_r;
    _ldb.compute(lpatch, kps, descp_l);
    _ldb.compute(rpatch, kps, descp_r);

    // Merging the descriptors
    desc = cv::Mat::zeros(1, desc_size * 2, CV_8U);
    descp_l.copyTo(desc.colRange(0, desc_size));
    descp_r.copyTo(desc.colRange(desc_size, desc_size * 2));
}

void PHOGDescriptor::getHistogram(const cv::Mat& edges, const cv::Mat& ors, const cv::Mat& mag, int startX, int startY, int width, int height, cv::Mat& hist)
{
    // Find and increment the right bin/s
    for (int x = startX; x < startX + height; x++)
    {
        for (int y = startY; y < startY + width; y++)
        {
            if (edges.at<uchar>(x,y) > 0)
            {
                int bin = (int)std::floor(ors.at<float>(x, y));
                hist.at<float>(0, bin) = hist.at<float>(0, bin) + mag.at<float>(x, y);
            }
        }
    }
}

void PHOGDescriptor::describe(const cv::Mat& image, cv::Mat& desc)
{
    int nbins = 60; // 20 bins as default.

	_desc_size = nbins + 4 * nbins + 16 * nbins;

    cv::Mat img = image;
    if (img.channels() > 1)
    {
        // Convert the image to grayscale
        cv::cvtColor(img, img, CV_BGR2GRAY);
    }

    // Mean and Standard Deviation
    cv::Scalar cvMean;
    cv::Scalar cvStddev;
    cv::meanStdDev(img, cvMean, cvStddev);
    double mean = cvMean(0);

    // Apply Canny Edge Detector
    cv::Mat edges;
    // Reduce noise with a kernel 3x3
    cv::blur(img, edges, cv::Size(3,3));
    // Canny detector
    cv::Canny(edges, edges, 0.66 * mean, 1.33 * mean);

    //  Computing the gradients.
    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;

    // Gradient X
    cv::Sobel(img, grad_x, CV_32F, 1, 0, 3);

    // Gradient Y
    cv::Sobel(img, grad_y, CV_32F, 0, 1, 3);

    // Total Gradient (approximate)
    cv::Mat grad_m = cv::abs(grad_x) + cv::abs(grad_y);

    // Computing orientations
    cv::Mat grad_o;
    cv::phase(grad_x, grad_y, grad_o, true);

    // Quantizing orientations into bins.
    double w = 360.0 / (double)nbins;
    grad_o = grad_o / w;

    // Creating the descriptor.
    desc = cv::Mat::zeros(1, nbins + 4 * nbins + 16 * nbins, CV_32F);
    int width = image.cols;
    int height = image.rows;

    // Level 0
    cv::Mat chist = desc.colRange(0, nbins);
    getHistogram(edges, grad_o, grad_m, 0, 0, width, height, chist);

    // Level 1
    chist = desc.colRange(nbins, 2 * nbins);
    getHistogram(edges, grad_o, grad_m, 0, 0, width / 2, height / 2, chist);

    chist = desc.colRange(2 * nbins, 3 * nbins);
    getHistogram(edges, grad_o, grad_m, 0, width / 2, width / 2, height / 2, chist);

    chist = desc.colRange(3 * nbins, 4 * nbins);
    getHistogram(edges, grad_o, grad_m, height / 2, 0, width / 2, height / 2, chist);

    chist = desc.colRange(4 * nbins, 5 * nbins);
    getHistogram(edges, grad_o, grad_m, height / 2, width / 2, width / 2, height / 2, chist);

    // Level 2
    int wstep = width / 4;
    int hstep = height / 4;
    int binPos = 5; // Next free section in the histogram
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            chist = desc.colRange(binPos * nbins, (binPos + 1) * nbins);
            getHistogram(edges, grad_o, grad_m, i * hstep, j * wstep, wstep, hstep, chist);
            binPos++;
        }
    }

    // Normalizing the histogram.
    cv::Mat_<float> sumMat;
    cv::reduce(desc, sumMat, 1, CV_REDUCE_SUM);
    float sum = sumMat.at<float>(0, 0);
    desc = desc / sum;
}

double GlobalDescriptor::dist(const cv::Mat& a, const cv::Mat& b, GlobalDescriptor *desc)
{
    double response;
    if (desc->getType() == GDESCRIPTOR_WISIFT || desc->getType() == GDESCRIPTOR_WISURF)
    {
        response = distEuclidean(a, b);
    }
    else if (desc->getType() == GDESCRIPTOR_PHOG)
    {
        //response = distBhattacharyya(a, b);
        response = distChiSquare(a, b);
    }
    else
    {
        response = double(distHamming(a.ptr(0), b.ptr(0), desc->getDescSize()));
    }
    return response;
}

double GlobalDescriptor::dist(const cv::Mat& a, const cv::Mat& b, const cv::Mat& icovar)
{
    return distMahalanobis(a, b, icovar);
}

GlobalDescriptor* GlobalDescriptor::create(const std::string& name, const GlobalDescriptorParams& params)
{
    GlobalDescriptor* desc = 0;

    if (name == "WI-SIFT")
	{
        desc = new WISIFTDescriptor(params);
	}
    else if (name == "WI-SURF")
	{
        desc = new WISURFDescriptor(params);
    }
    else if (name == "BRIEF-Gist")
    {
        desc = new BRIEFGistDescriptor(params);
    }
    else if (name == "WI-LDB")
    {
        desc = new WILDBDescriptor(params);
    }
    else if (name == "PHOG")
    {
        desc = new PHOGDescriptor(params);
    }

    return desc;
}

}
