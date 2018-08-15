/**
 * Class Plotdata
 * 
 * Stores data for one 2D plotting axis
 *
 * Provides operators and mathematical functions to map a Plotdata
 * object to another using unary and binary functions.
 *
 * Provides insertion and extraction of special commands into and from
 * Plotdata pairs as static operations.
 * Available commands are:
 *      colourChange (to change the foreground colour of graphs)
 *      singlePoint  (to draw a single point of data as a special marker)
 *
 * Author: 	jlk
 * Version:	1.1
 * Date:	July 2005
 * 
 * This file is in the public domain and can be used without any
 * restriction.
 */
 
#ifndef Plotdata_H
#define Plotdata_H

#include <cmath>
#include <vector>
#include <iostream>

using namespace std;


// Unary function pointer type accepted by Plotdata (double parameter)
typedef double (*Func)(double operand);
#define func_ref Func &

// Binary function pointer type accepted by Plotdata
typedef double (*BinFunc)(double operand1, double operand2);
#define binfunc_ref BinFunc & 

/** When true defines a range to be logarithmic */
typedef bool LogSpace;

/** Define data iterator */
typedef vector<double>::const_iterator  dataIterator;

/** Define range space: LOGarithmic or LINear */
typedef enum RangeSpace
{
	LIN = false,
	LOG = true
};

/** Remove case sensitivity for Plotdata class */
class Plotdata;
typedef Plotdata plotdata;

/** #define Plotdata & as a type (avoid using confusing '&' symbols in C) */
#define plotdata_ref plotdata & 
#define Plotdata_ref Plotdata & 

/** Number of points in a plot */
enum Grain
{
	EXTRA_FINE = 901,
	FINE = 501,
	MEDIUM = 301,
	COARSE = 201,
	GROSS = 101
};

const double NOPLOT = nan(""); // Any point of this value will not be plotted

const int NOCOLOR      = -1;
const int RESETCOLOR   = -2;

class Plotdata
{
    // --------------------------------------------------------------
    //     OPERATORS
        
	friend ostream & operator << (ostream & out, const Plotdata & pd);
	friend istream & operator >> (istream & in, Plotdata & pd);
	friend Plotdata operator + (double op1, const Plotdata & pd);
	friend Plotdata operator - (double op1, const Plotdata & pd);
	friend Plotdata operator * (double op1, const Plotdata & pd);
	friend Plotdata operator / (double op1, const Plotdata & pd);
	      
public:

	Plotdata operator +  (const Plotdata &) const;
	Plotdata operator +  (double) const;
	Plotdata operator -  (const Plotdata &) const;
	Plotdata operator -  (double) const;
	Plotdata operator *  (const Plotdata &) const;
	Plotdata operator *  (double) const; 
	Plotdata operator /  (const Plotdata &) const;
	Plotdata operator /  (double) const;
	Plotdata operator ^  (double) const;
    Plotdata operator -  ( ) const;
	Plotdata & operator << (const Plotdata &); // concatenate
	Plotdata & operator << (double); // add a double to the data

    // --------------------------------------------------------------
    // Constructors
	inline Plotdata(): data(MEDIUM), userFunction(0),
                       userBinFunction(0){}
		   Plotdata(double min, double max);
		   Plotdata(double min, double max, Grain grain);
           Plotdata(const double *array, int dataSize);
	inline Plotdata(size_t s): data(s), userFunction(0),
                               userBinFunction(0){}
	inline Plotdata(vector<double> d):  data(d), 
                                        userFunction(0), userBinFunction(0){};
    // Member Functions
    void insert(const double array[], int dataSize);
	inline size_t size() const {return data.size();}
	inline void point(double p) {data.push_back(p);}
	void plotRange(double min, double max, 
				   int numPoints, LogSpace isLog = false);

	double min( )const;
	double max( )const;
	inline void clear( ) {data.clear();}
	inline const vector<double> & getData() const{return data;}
	inline Func & userfunc() { return userFunction; }
	inline BinFunc & userBinfunc() { return userBinFunction; }

    Plotdata doFunc(Func aFunction) const;
	Plotdata doBinFunc(BinFunc aFunction, double operand2) const;
	Plotdata doBinFunc(double operand2) const;

    // --------------------------------------------------------------
    // Class (static) functions
    
    // Retrieves the maximum of each of x and y in a Plotdata pair
    // Values corresponding to a NAN in the other Plotdata are not included
    static void max(const Plotdata &x, const Plotdata &y,
                    double &xMax, double &yMax);
    // Retrieves the minimum of each of x and y in a Plotdata pair
    // Values corresponding to a NAN in the other Plotdata are not included
    static void min(const Plotdata &x, const Plotdata &y,
                    double &xMin, double &yMin);
    // Retrieve the single point coordinates requested by special command
    static bool singlePoint(double &xCoord, double &yCoord, 
                            dataIterator &x, dataIterator &y);
    // Insert singlePoint coordinates into plot data pair
    static void singlePoint( Plotdata &x, Plotdata &y, 
                             double xCoord, double yCoord);
    // Retrieve the colour change requested by special command
    static int colorChange(dataIterator &x, dataIterator &y);
    // Insert special command for a colour change
    static void colorChange(Plotdata &x, Plotdata &y, int colour);
    // Insert special command to reset colour to before last change
    static void colorReset(Plotdata &x, Plotdata &y);
    // Return true if theColour is a valid colour or RESETCOLOR
    static bool isColor(int theColour);
    // Insert a single (invisible) point in plotdata pair
    static void soliton(Plotdata &x, Plotdata &y, double xval, double yval );
    

private:
	vector<double> data;
	Func userFunction; // Any user-designed unary function
	BinFunc userBinFunction; // Any user-designed binary function
};

#endif

