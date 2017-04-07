#include <fstream>
#include <cfloat>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;

class point_and_box{
public:
	int idx;
	int box;
	point_and_box(int arg_idx){ idx = arg_idx; box = -1;}
	bool operator < (const point_and_box &rhs) const {return(box < rhs.box);}
};

template <class T> class vec3{
public:
    T x, y ,z;
 
     vec3(){    }
    
    
    vec3(T arg_x, T arg_y, T arg_z){
        x = arg_x;
        y = arg_y;
        z = arg_z;
    }
    
    vec3<T>& operator = (const vec3<T> &rhs) { 
        x = rhs.x; 
        y = rhs.y;
        z = rhs.z;
        return *this;
    }
    
    vec3(const vec3<T> & rhs)
    {
        x = rhs.x; 
        y = rhs.y;
        z = rhs.z;
    }
    
    vec3<T>& operator+=(const vec3<T>& rhs){
        x += rhs.x; 
        y += rhs.y;
        z += rhs.z;
        return *this;
    
    } 
    
    
};

void getMinMax(vector< vec3<double> > & inCloud, vec3<double> &minp, vec3<double> &maxp){
	for(int i = 0; i < inCloud.size(); i++){
		minp.x = std::min(minp.x, inCloud[i].x);
		minp.y = std::min(minp.y, inCloud[i].y);
		minp.z = std::min(minp.z, inCloud[i].z);
	
		maxp.x = std::max(maxp.x, inCloud[i].x);
		maxp.y = std::max(maxp.y, inCloud[i].y);
		maxp.z = std::max(maxp.z, inCloud[i].z);
	}

}


void filterPoints(vec3<int> & leafCount, vector< vec3<double> > & inCloud, vector< vec3<double> > & outCloud, vector< point_and_box > indices){



	
	
	//Compute minimum and maximum point values
	vec3<double> minp(DBL_MAX, DBL_MAX, DBL_MAX), maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
	getMinMax(inCloud, minp, maxp);

	//Compute bounding box sizes
	vec3<double> leafSize((maxp.x - minp.x)/leafCount.x, (maxp.y - minp.y)/leafCount.y, (maxp.z - minp.z)/leafCount.z);
	vec3<double> inv_leafSize(1.0/leafSize.x, 1.0/leafSize.y, 1.0/leafSize.z);

	//Compute the minimum and maximum bounding box values
	vec3<int> minb( static_cast<int> ( floor(minp.x * inv_leafSize.x) ),
	                static_cast<int> ( floor(minp.y * inv_leafSize.y) ),
	                static_cast<int> ( floor(minp.z * inv_leafSize.z) ) ); 
	                
    vec3<int> maxb( static_cast<int> ( floor(maxp.x * inv_leafSize.x) ),
                    static_cast<int> ( floor(maxp.y * inv_leafSize.y) ),
                    static_cast<int> ( floor(maxp.z * inv_leafSize.z) ) ); 

	vec3<int> divb(maxb.x - minb.x + 1, maxb.y - minb.y + 1, maxb.z - minb.z + 1);
	vec3<int> divb_mul(1, divb.x, divb.x * divb.y);

	//Go over all points and insert them into the right leaf
	for(int i = 0; i < inCloud.size(); i++){
		int ijk0 = static_cast<int> ( floor(inCloud[i].x * inv_leafSize.x) - minb.x );
		int ijk1 = static_cast<int> ( floor(inCloud[i].y * inv_leafSize.y) - minb.y );
		int ijk2 = static_cast<int> ( floor(inCloud[i].z * inv_leafSize.z) - minb.z );
		int idx = ijk0 * divb_mul.x + ijk1 * divb_mul.y + ijk2 * divb_mul.z;
		indices[i].box = idx;
	}
	sort(indices.begin(), indices.end(), less<point_and_box>());

	for(int cp = 0; cp < inCloud.size();){
	
	    vec3<double> centroid(inCloud[indices[cp].idx].x, inCloud[indices[cp].idx].y, inCloud[indices[cp].idx].z);
		int i = cp + 1;
		while(i < inCloud.size() && indices[cp].box == indices[i].box){
            centroid += inCloud[indices[cp].idx];
			++i;
		}
		centroid.x /= static_cast<double>(i - cp);
		centroid.y /= static_cast<double>(i - cp);
		centroid.z /= static_cast<double>(i - cp);
		
		outCloud.push_back(centroid);
		cp = i;
	}
	
}

int main(int argc, char* argv[]){
	vec3<double> point(0.0,0.0,0.0);
	
	vector< vec3<double> > inCloud, outCloud;
	vector<point_and_box> indices;
	

	ifstream infile(argv[1]);
	
	while(infile >> point.x >> point.y >> point.z){
	    indices.push_back( point_and_box( inCloud.size() ) );
        inCloud.push_back( vec3<double>(point) );
	}
	infile.close();
	vec3<int> leafCount;
	cout << "There are " << inCloud.size() << " points. Specify the" << endl;
    cout << "number of the leaves in X axis: ";
    cin  >> leafCount.x;
    cout << "number of the leaves in Y axis: ";
    cin  >> leafCount.y;
    cout << "number of the leaves in Z axis: ";
    cin  >> leafCount.z;

	filterPoints(leafCount, inCloud, outCloud, indices);
	cout << "The number of points are downsampled to " << outCloud.size() <<"." << endl;
	ofstream outfile(argv[2]);
	for(int i = 0; i < outCloud.size(); i++)
		outfile << outCloud[i].x << "  " << outCloud[i].y << "  " << outCloud[i].z << endl;
	outfile.close();
}
