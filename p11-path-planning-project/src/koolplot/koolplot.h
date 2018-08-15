/**
 * Header file to the "koolplot" library
 * 
 * Provides basic, but also very simple-to-use 2D data plotting for
 * C and C++ programs using the WinBGIm graphics library.
 *
 * Example:
 * #include <koolplot.h>
 * int main()
 * {
 * 		plotdata x(0, 360), y = sin(x * M_PI / 180);
 *		plot(x, y);
 *		return 0;
 * }
 *
 * The "plot" function opens a window and displays the graph. The window
 * is closed when any key is pressed.
 *
 * Author: 	jlk
 * Version:	1.1
 * Date:	July 2005
 * 
 * This file is in the public domain and can be used without any
 * restriction.
 */
 
#ifndef KOOLPLOT_H
#define KOOLPLOT_H

#include "Plotstream.h"

/*
   The following colour are already defined:
    RGB( 0, 0, 0 );         // BLACK
    RGB( 0, 0, 128);        // BLUE
    RGB( 0, 128, 0 );       // GREEN
    RGB( 0, 128, 128 );     // CYAN
    RGB( 128, 0, 0 );       // RED
    RGB( 128, 0, 128 );     // MAGENTA
    RGB( 128, 128, 0 );     // BROWN
    RGB( 192, 192, 192 );   // LIGHTGREY
    RGB( 128, 128, 128 );   // DARKGREY
    RGB( 128, 128, 255 );   // LIGHTBLUE
    RGB( 128, 255, 128 );   // LIGHTGREEN
    RGB( 128, 255, 255 );   // LIGHTCYAN
    RGB( 255, 128, 128 );   // LIGHTRED
    RGB( 255, 128, 255 );   // LIGHTMAGENTA
    RGB( 255, 255, 0 );     // YELLOW
    RGB( 255, 255, 255 );   // WHITE

  Below, define some extra web colours useful in graphs
*/

const int CRIMSON           = COLOR(220, 20, 60);
const int CHOCOLATE         = COLOR(210, 105, 30);
const int GOLDENROD         = COLOR(218, 165, 32);
const int DARKORANGE        = COLOR(255, 140, 0);
const int REDRED            = COLOR(255, 0, 0);
const int DARKSLATEGRAY     = COLOR(47, 79, 79);
const int DEEPPINK          = COLOR(255, 20, 147);
const int TURQUOISE         = COLOR(64, 224, 208);
const int DARKVIOLET        = COLOR(148, 0, 211);
const int BEIGE             = COLOR(245, 245, 220);
const int MEDIUMSEAGREEN    = COLOR(60, 179, 113);
const int LIMEGREEN         = COLOR(50, 205, 50);
const int DARKGREEN         = COLOR(0, 100, 0);
const int MEDIUMBLUE        = COLOR(0, 0, 205);
const int BLUEBLUE          = COLOR(0, 0, 255);
const int DODGERBLUE        = COLOR(30, 144, 255);

/* -------------------------------------------------------- */

/**
 * Plot x..y data
 */
void plot(const plotdata_ref x, const plotdata_ref y); 

/**
 * Plot x..y data, specify curve colour
 */
void plot(const plotdata_ref x, const plotdata_ref y, int colour); 

/**
 * Plot x..y data, specify label
 */
void plot(const plotdata_ref x, const plotdata_ref y, char *label); 

/**
 * Plot x..y data, specify label and curve colour
 */
void plot(const plotdata_ref x, const plotdata_ref y, char *label, int colour); 

/**
 * Plot x..y data, specify curve colour, then label
 */
void plot(const plotdata_ref x, const plotdata_ref y, int colour, char *label); 

/**
 * Plot f(x) function (use together with f(x) function)
 */
void plot(const plotdata_ref x, func_ref f); 

/**
 * Plot f(x) function, specify curve colour (use together with f(x) function)
 */
void plot(const plotdata_ref x, func_ref f, int colour); 

/**
 * Plot f(x) function, specify label (use together with f(x) function)
 */
void plot(const plotdata_ref x, func_ref f, char *label); 

/**
 * Plot f(x) function, specify label and curve colour (use together with f(x) function)
 */
void plot(const plotdata_ref x, func_ref f, char *label, int colour); 

/**
 * Plot f(x) function, specify curve colour, then label (use together with f(x) function)
 */
void plot(const plotdata_ref x, func_ref f, int colour, char *label); 

/**
 * Installs a user-defined unary function of x as f(x)
 * example: plotdata x(-6, 6);
 *			f(x) = sinc;	// if sinc is a function defined by the user.
 *			plot(x, f(x));	// Plots the graph of sinc(x) from -6 to 6
 */
func_ref  f(plotdata_ref x);

/**
 * Installs a user-defined binary function of x as f2(x)
 * example: plotdata x(-270, 270);
 *			f2(x) = tanLessThan; // tanLessThan is a user-defined binary function.
 */
binfunc_ref f2(plotdata_ref x);

/**
 * Calculates a user-defined binary function of x as y = f2(x, double_value)
 * example: plotdata x(-270, 270);
 *			f2(x) = tanLessThan; //  tanLessThan -user-defined binary function.
 *			plot(x, f2(x, 10));	 // Plots the graph of tanLessThan(x, 10)
 */
plotdata f2(const plotdata_ref x, double op2);

/**
 * Adds next point to plotting data
 * -You add x and y coordinates in 2 separate plotdatas then call plot(). 
 * plot() will draw the curve joining all the data points. 
 * 
 * @param pd  the plotdata to add the point to
 * @param val value of the point to add
 */

void point(plotdata_ref pd, double val);

/**
 * Adds next point to plotting data
 *  
 * Same as above, but will take both axes and coordinates in one call. 
 * 
 * @param xAxis the x plotdata to add the point to
 * @param xCoord value of the x coordinate to add
 * @param yAxis the y plotdata to add the point to
 * @param yCoord value of the y coordinate to add
 */
void point(plotdata_ref xAxis, plotdata_ref yAxis,
           double xCoord, double yCoord);

/**
 * Insert array of data points into plotting data
 *  
 * @param axis the  plotdata to add the points to
 * @param array The array of points to insert
 * @param numToInsert the number of points to add from the array.
 */
void insert(plotdata_ref axis, const double array[], int numToInsert);

/** Sets the bottom left corner of the graph axes 
  * xmin and ymin should be less than or equal to any coordinate
  * on the graph.
  */ 
void axesBotLeft(plotdata_ref x, plotdata_ref y, 
                 double xmin, double ymin);

/** Sets the top right corner of the graph axes
  * xmax and ymax should be larger than or equal to any coordinate
  * on the graph.
  */ 
void axesTopRight(plotdata_ref x, plotdata_ref y, double xmax, double ymax);

/** Adds visible mark to plot, centered on coordinates xpos, ypos */
void addMark(plotdata_ref x, plotdata_ref y, double xpos, double ypos);

/** Adds visible mark at  coordinates xpos, ypos, specify colour */
void addMark(plotdata_ref x, plotdata_ref y, double xpos, double ypos, int colour);

/** Set colour of graph to requested colour */
void setColor(plotdata_ref x, plotdata_ref y, int colour);

/** Reset colour of graph to last colour before change */
void resetColor(plotdata_ref x, plotdata_ref y);

/**
 * Clear previous data from an axis.
 * @param the axis to clear data from
 */
void clear(plotdata_ref pd);

/**
 * Breaks the current plot data at this point.
 *
 * Later, when plotting, the last point of the data previously 
 * entered will not be joined to the first point of the next data.
 * This allows plotting more than one function on a single graph
 * using the "point" method of data entry.
 * @param xAxis the x data to break
 * @param yAxis the y data to break
 */
 void breakplot(plotdata_ref x, plotdata_ref y);

/**
 * Print plotting data on axis "anAxis" to standard output (for debugging)
 */
void printplotdata(const plotdata_ref anAxis);


/**********************************************************************/
/*________ Maths functions that may be used to define functions ______*/

/*
 * Return new data, the sine of the original data
 *
 * @param pd the original plotdata
 */
plotdata sin(const plotdata_ref pd);

/**
 * Return new data, the cosine of the original data
 *
 * @param pd the original plotdata
 */
plotdata cos(const plotdata_ref pd);

/**
 * Return new data, the tan of the original data
 *
 * @param pd the original plotdata
 */
plotdata tan(const plotdata_ref pd);

/**
 * Return new data, the asin of the original data
 *
 * @param pd the original plotdata
 */
plotdata asin(const plotdata_ref pd);

/**
 * Return new data, the acos of the original data
 *
 * @param pd the original plotdata
 */
plotdata acos(const plotdata_ref pd);


/**
 * Return new data, the atan of the original data
 *
 * @param pd the original plotdata
 */
plotdata atan(const plotdata_ref pd);

/**
 * Return new data, the hyperbolic sine of the original data
 *
 * @param pd the original plotdata
 */
plotdata sinh(const plotdata_ref pd);

/**
 * Return new data, the hyperbolic cosine of the original data
 *
 * @param pd the original plotdata
 */
plotdata cosh(const plotdata_ref pd);

/**
 * Return new data, the hyperbolic tan of the original data
 *
 * @param pd the original plotdata
 */
plotdata tanh(const plotdata_ref pd);

/**
 * Return new data, the square root of the original data
 *
 * @param pd the original plotdata
 */
plotdata sqrt(const plotdata_ref pd);

/**
 * Return new data, the absolute value of the original data
 *
 * @param pd the original plotdata
 */
plotdata fabs(const plotdata_ref pd);

/**
 * Return new data, the natural logarithm of the original data
 *
 * @param pd the original plotdata
 */
plotdata log(const plotdata_ref pd);

/**
 * Return new data, the log (base 10)l of the original data
 *
 * @param pd the original plotdata
 */
plotdata log10(const plotdata_ref pd);

/**
 * Return new data, the exponential of the original data
 *
 * @param pd the original plotdata
 */
plotdata exp(const plotdata_ref pd);

/**
 * Return new data, the power "exp" of the original data
 *
 * @param pd  the original plotdata
 * @param exp value of the exponent to raise the data to.
 */
plotdata pow(const plotdata_ref pd, double exp);

#endif

