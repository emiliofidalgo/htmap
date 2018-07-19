#ifndef _GLOBALDESCRIPTOR_H_
#define _GLOBALDESCRIPTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "hamap/util/Util.h"
#include "ldb.h"

namespace hamap
{

enum GlobalDescriptorType
{
    GDESCRIPTOR_WISIFT,
    GDESCRIPTOR_WISURF,
    GDESCRIPTOR_BRIEFGIST,
    GDESCRIPTOR_WILDB,
    GDESCRIPTOR_PHOG
};

// ---
// General parameters for global descriptors
// ---
struct GlobalDescriptorParams
{
    GlobalDescriptorParams()
	{}
};

// ---
// Abstract global descriptor class.
// ---
class GlobalDescriptor
{
	public:
        GlobalDescriptor(const GlobalDescriptorType& type, const int nbytes) :
			_type(type),
			_desc_size(nbytes)
        {}
        virtual ~GlobalDescriptor()
        {}

        inline int getDescSize() { return _desc_size; }
        inline GlobalDescriptorType getType() { return _type; }
        static double dist(const cv::Mat& a, const cv::Mat& b, GlobalDescriptor* desc);
        static double dist(const cv::Mat& a, const cv::Mat& b, const cv::Mat& icovar);

        static GlobalDescriptor* create(const std::string& name, const GlobalDescriptorParams& params);

        virtual void parseParameters(const GlobalDescriptorParams& params) = 0;
        virtual void describe(const cv::Mat& image, cv::Mat& desc) = 0;

	protected:
        GlobalDescriptorType _type;
		int _desc_size;
};

// ---
// WI-SIFT descriptor class.
// ---
class WISIFTDescriptor : public GlobalDescriptor
{
	public:
        WISIFTDescriptor(const GlobalDescriptorParams& params) :
            GlobalDescriptor(GDESCRIPTOR_WISIFT, 256)
	{			
			parseParameters(params);
    }

    void parseParameters(const GlobalDescriptorParams& params)
	{		
    }

    void describe(const cv::Mat& image, cv::Mat& desc);
};

// ---
// WI-SURF descriptor class.
// ---
class WISURFDescriptor : public GlobalDescriptor
{
    public:
        WISURFDescriptor(const GlobalDescriptorParams& params) :
            GlobalDescriptor(GDESCRIPTOR_WISURF, 256)
    {
            parseParameters(params);
    }

    void parseParameters(const GlobalDescriptorParams& params)
    {
    }

    void describe(const cv::Mat& image, cv::Mat& desc);
};

// ---
// BRIEF-Gist descriptor class.
// ---
class BRIEFGistDescriptor : public GlobalDescriptor
{
    public:
        BRIEFGistDescriptor(const GlobalDescriptorParams& params) :
            GlobalDescriptor(GDESCRIPTOR_BRIEFGIST, 64)
    {
            parseParameters(params);
    }

    void parseParameters(const GlobalDescriptorParams& params)
    {
    }

    void describe(const cv::Mat& image, cv::Mat& desc);
};

// ---
// WI-LDB descriptor class.
// ---
class WILDBDescriptor : public GlobalDescriptor
{
    public:
        WILDBDescriptor(const GlobalDescriptorParams& params) :
            GlobalDescriptor(GDESCRIPTOR_WILDB, 64)
    {
            parseParameters(params);
    }

    void parseParameters(const GlobalDescriptorParams& params)
    {
    }

    void describe(const cv::Mat& image, cv::Mat& desc);

    private:
        LdbDescriptorExtractor _ldb;
};

// ---
// PHOG descriptor class.
// ---
class PHOGDescriptor : public GlobalDescriptor
{
    public:
        PHOGDescriptor(const GlobalDescriptorParams& params) :
            GlobalDescriptor(GDESCRIPTOR_PHOG, 420) // 20 Bins (18 deg per bin), and L = 3
    {
            parseParameters(params);
    }

    void parseParameters(const GlobalDescriptorParams& params)
    {
    }

    void describe(const cv::Mat& image, cv::Mat& desc);

private:
    void getHistogram(const cv::Mat& edges, const cv::Mat& ors, const cv::Mat& mag, int startX, int startY, int width, int height, cv::Mat& hist);
};

}

#endif /* GLOBALDESCRIPTOR_H_ */
