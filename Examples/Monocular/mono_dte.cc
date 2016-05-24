//
// Created by dmitry on 5/24/16.
// DTE frame reader
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

#include"System.h"

using namespace std;
using namespace boost::filesystem;

int LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./mono_dte path_to_vocabulary path_to_settings path_to_sequence" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

  auto nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat im;
  for(auto ni=0; ni<nImages; ni++)
  {
    // Read image from file
    im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni];

    if(im.empty())
    {
      cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
      return 1;
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni]=static_cast<float>(ttrack);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for(int ni=0; ni<nImages; ni++)
  {
    totaltime+=vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  cout << "mean tracking time: " << totaltime/nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

int LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
  path pathToSequence(strPathToSequence);
  double time = 0;

  try
  {
    if (exists(pathToSequence) && is_directory(pathToSequence))    // does p actually exist?
    {
      typedef vector<path> vec;
      vec filePaths;

      copy(directory_iterator(pathToSequence), directory_iterator(), back_inserter(filePaths));
      sort(filePaths.begin(), filePaths.end());

      for (auto p : filePaths) //just get all images
      {
        if(is_regular_file(p) && (extension(p).compare(".png") || extension(p).compare(".bmp")))
        {
          vstrImageFilenames.push_back(p.c_str());
          vTimestamps.push_back(time++);
        }
      }
    }
    else
      cout << pathToSequence << " directory does not exist\n";
  }
  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

  return 0;
}