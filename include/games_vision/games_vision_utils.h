#ifndef GAMES_VISION_UTILS_H
#define GAMES_VISION_UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>          // for std::sstreams
#include <fstream>          // for std::sstreams
#include <limits>
#include "vision_utils/hausdorff_distances.h"

/*!
 \param path
 \return std::string
 \example path="/tmp/foo/bar.dat", returns "/tmp/foo/"
 \example path="foo/bar.dat", returns "foo/"
 \example path="/bar.dat", returns "/"
 \example path="bar.dat", returns ""
*/
inline std::string extract_folder_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return "";
  return path.substr(0, 1 + slash_pos);
} // end extract_folder_from_full_path()

////////////////////////////////////////////////////////////////////////////////

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer - one line in the vector for each file line
 */
inline bool retrieve_file_split(const std::string & filepath,
                                std::vector<std::string> & ans,
                                bool remove_empty_lines = false,
                                bool remove_empty_last_line = true) {
  printf("retrieve_file_split('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(filepath.c_str(), std::ios::in);
  if (!myfile.is_open()) {// error while reading the file
    printf("Unable to open file '%s'", filepath.c_str());
    return false;
  }
  std::string line;
  ans.clear();
  while (myfile.good()) {
    getline(myfile, line);
    if (remove_empty_lines && line.empty())
      continue;
    ans.push_back(line);
  } // end myfine.good()
  if (remove_empty_last_line && ans.back().empty())
    ans.pop_back();
  myfile.close();
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   get a vector from an image with all the non nul points
 */
template<class Pt2Iterable>
inline void nonNulPoints(const cv::Mat1b & img, Pt2Iterable & rep) {
  printf("nonNulPoints()");
  rep.resize( cv::countNonZero(img) );
  int rep_idx = 0;
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++) {
        rep[rep_idx].x = col;
        rep[rep_idx].y = row;
        ++rep_idx;
      }
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////////

/*! Convert a vector to a fancy string
 \param vector
    anything that can be accesed with iterators
 \return std::string the representation of this vector, separated by ;
 */
template<class _Iterable>
inline std::string iterable_to_string(const _Iterable & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  typename _Iterable::const_iterator it = obj.begin();
  while (it != obj.end()) {
    ans_stream << *it++;
    if (it!= obj.end())
      ans_stream << "; ";
  }
  ans_stream << "]";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

template<class _Pt2>
inline double dist_L1_int(const _Pt2 & a, const _Pt2 & b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

/*!
 * \brief   the hausdorf d22 distance, modified to accept a max value
 *
 * \param   A a vector of _Pt2
 * \param   B a vector of _Pt2
 * \param   min the min distance for which we will return INFINITY
 * \return  max(d6 (A, B), d6 (B, A) )
            = max( 1 / |A| * sum_{a in A} d(a, B) ,
                   1 / |B| * sum_{b in B} d(b, A))
                                                    if < min,
            INFINITY                                else

            It is between 0      (perfect match)
            and d6 (a0, b0)      (completely different)
                                 where a0 and b0 are the most remote points
                                 of A and B.
 */
template<class _Pt2, class Pt2Iterable>
inline double
D22_with_min(const Pt2Iterable & A, const Pt2Iterable & B,
             const double min,
             double (*dist_func_ptr)(const _Pt2 &, const _Pt2 &)
             = &dist_L1_int) {

  unsigned int nA = A.size(), nB = B.size();
  double min_A = min * nA, sum_A = 0;
  for (unsigned int idx_A = 0; idx_A < nA; ++idx_A) {
    sum_A += vision_utils::dist_pt_set( A[idx_A], B, dist_func_ptr);
    if (sum_A > min_A)
      return std::numeric_limits<typename _Pt2::value_type>::max();
  } // end loop A
  sum_A = sum_A / nA;

  double min_B = min * nB, sum_B = 0;
  for (unsigned int idx_B = 0; idx_B < nB; ++idx_B) {
    sum_B += vision_utils::dist_pt_set( B[idx_B], A, dist_func_ptr);
    if (sum_B > min_B)

      return std::numeric_limits<typename _Pt2::value_type>::max();
  } // end loop B
  sum_B = sum_B / nB;

  return std::max(sum_A, sum_B);
}

////////////////////////////////////////////////////////////////////////////////

//! \retun true if \a small is included into \a big
template<class Rect>
inline bool bboxes_included(const Rect & big, const Rect & small) {
  if (big.x > small.x)
    return 0;
  if (big.y > small.y)
    return 0;
  if (big.x + big.width < small.x + small.width)
    return 0;
  if (big.y + big.height < small.y + small.height)
    return 0;
  return 1;
}

////////////////////////////////////////////////////////////////////////////////


#endif // GAMES_VISION_UTILS_H
