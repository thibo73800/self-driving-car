/**
 * BGI_util.h
 *
 * Declares some utility WinBGIm functions
 *
 * @author jlk
 * @version 1.0 - August 2005
 */
 

/**
 * Synchronises the active and visual Window buffers
 * This function called after every major drawing will ensure
 * smooth, flicker-free drawing with double-buffering.
 */
void synchronize();

