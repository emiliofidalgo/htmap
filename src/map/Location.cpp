#include "hamap/map/Location.h"

namespace hamap
{

Location::Location(const int& max_images, const int& feat_per_img, const int& desc_bytes) :
    init(false),
    id(0),
    _bindex(0),
    _params(0)
{
    vi::BinaryIndexParams bparams;
    bparams.max_descriptors = (max_images * feat_per_img * 3);
    bparams.desc_bytes = desc_bytes;
    _bindex = new vi::BinaryIndex(bparams);
    _params = Params::getInstance();
}

Location::~Location()
{
    delete _bindex;
}

void Location::addImage(const int& id, const std::string& image_filename)
{
    if (!init)
    {
        initializeLocation(id, image_filename);
        init = true;
    }
    else
    {
	    Image curr_img;
	    curr_img.load(image_filename);
	        
		LocationEntry locent(id, image_filename);
	    images.push_back(locent);

		if (images.size() < _params->loc_max_images)
		{

	        // Updating the representative of the location
	        // Copying the descriptor
	        gdescriptors.push_back(curr_img.gdsc);

	        // Computing the mean and the inv cov matrix.
	        updateGlobalDescription();

	        // Matching the images
	        std::vector<std::vector<cv::DMatch> > matches_feats;
	        _bindex->search(curr_img.dscs, matches_feats, 2);

	        // Filtering matches
	        std::vector<cv::DMatch> matches;
	        for (unsigned m = 0; m < matches_feats.size(); m++)
    	    {
	            if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8)
    	        {
        	        matches.push_back(matches_feats[m][0]);
	            }
    	    }        

        	// Updating the index
	        _bindex->update(int(images.size() - 1), curr_img.kps, curr_img.dscs, matches);        
		}
		else
		{
	        // Matching the images
	        std::vector<std::vector<cv::DMatch> > matches_feats;
	        _bindex->search(curr_img.dscs, matches_feats, 2);

	        // Filtering matches
	        std::vector<cv::DMatch> matches;
	        for (unsigned m = 0; m < matches_feats.size(); m++)
    	    {
	            if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8)
    	        {
        	        matches.push_back(matches_feats[m][0]);
	            }
    	    }        

			_bindex->addToInvertedIndex(int(images.size() - 1), curr_img.kps, matches);
		}
    }
}

void Location::searchImages(const cv::Mat& dscs, std::map<int, double>& image_matches, double match_ratio)
{

    // Matching the current descriptors against the index
    std::vector<std::vector<cv::DMatch> > matches_feats;
    _bindex->search(dscs, matches_feats, 2);

    // Filtering the matchings
    std::vector<cv::DMatch> matches;
    for (unsigned m = 0; m < matches_feats.size(); m++)
    {
        if (matches_feats[m][0].distance < matches_feats[m][1].distance * match_ratio)
        {
            matches.push_back(matches_feats[m][0]);
        }
    }

    // Getting similar images according to the index and filtering the results.
    std::vector<vi::ImageMatch> imm;
    _bindex->getSimilarImages(dscs, matches, imm);

    #pragma omp parallel for
    for (unsigned i = 0; i < imm.size(); i++)
    {
        // Translating the index to the real image identifier.
        imm[i].image_id = images[imm[i].image_id].image_id;
        
		#pragma omp critical(IMMATCH)
        {
            image_matches[imm[i].image_id] = imm[i].score;
        }
    }
}

void Location::initializeLocation(const int& id, const std::string& image_filename)
{
    LocationEntry locent(id, image_filename);

    images.push_back(locent);    

    // Loading the image.
    Image img;
    img.load(image_filename);

    // Coyping the descriptor    
    img.gdsc.copyTo(gdescriptors);
    int dsize = _params->gdescriptor->getDescSize();
    desc = cv::Mat::zeros(1, dsize, CV_64F);
    invcov = cv::Mat::zeros(dsize, dsize, CV_64F);

    // Computing the mean and the inv cov matrix.
    updateGlobalDescription();

    // Adding the image to the index.
    _bindex->add(0, img.kps, img.dscs);
}

void Location::updateGlobalDescription()
{
    int dsize = _params->gdescriptor->getDescSize();
    for (int i = 0; i < dsize; i++)
    {
        cv::Mat col = gdescriptors.col(i);

        cv::Scalar meanV, stdevV;
        cv::meanStdDev(col, meanV, stdevV);

        double mean = (double)meanV[0];
        double stde = (double)stdevV[0];

        desc(0, i) = mean;
        if (stde > 0)
        {
            invcov(i, i) = 1.0 / stde;
        }
    }
}

unsigned Location::numImages()
{
    return images.size();
}

}
