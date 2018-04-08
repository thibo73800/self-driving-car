/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/*
		Initialize all particles to first position (based on estimates of
		x, y, theta and their uncertainties from GPS) and all weights to 1.
		A random gaussian is add to each particile
	*/
	default_random_engine gen;
	// First creates a normal (Gaussian) distribution for x, y and theta
	normal_distribution<double> gaussian_x(x, std[0]);
	normal_distribution<double> gaussian_y(y, std[1]);
	normal_distribution<double> gaussian_theta(theta, std[2]);

	// Create all particles
	for (size_t p = 0; p < this->num_particles; p++) {
		double sample_x, sample_y, sample_theta;
		// Create the new particles
		Particle n_particle = {};

		sample_x = gaussian_x(gen);
		sample_y = gaussian_y(gen);
		sample_theta = gaussian_theta(gen);

		n_particle.id = p;
		n_particle.x = sample_x;
		n_particle.y = sample_y;
		n_particle.theta = sample_theta;
		n_particle.weight = 1.0/this->num_particles;

		this->particles.push_back(n_particle);
	}
	this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	/*
		Add measurements to each particle and add random Gaussian noise.
	*/
	// Random Gaussian Noise
	default_random_engine gen;
	normal_distribution<double> gaussian_x(0., std_pos[0]);
	normal_distribution<double> gaussian_y(0., std_pos[1]);
	normal_distribution<double> gaussian_yaw(0., std_pos[2]);

	for (size_t p = 0; p < this->num_particles; p++) {
		float nx, ny, ntheta;
		float x = this->particles.at(p).x;
		float y = this->particles.at(p).y;
		float theta = this->particles.at(p).theta;

		if (yaw_rate == 0) {
			nx = x + velocity*cos(theta)*delta_t;
			ny = y + velocity*sin(theta)*delta_t;
			ntheta = theta;
		}
		else {
			nx = x + ((velocity/yaw_rate)*(sin(theta + (yaw_rate*delta_t)) - sin(theta)));
			ny = y + ((velocity/yaw_rate)*(cos(theta) - cos(theta + (yaw_rate*delta_t))));
			ntheta  = theta + yaw_rate*delta_t;
		}
		nx = nx + gaussian_x(gen);
		ny = ny + gaussian_y(gen);
		ntheta = ntheta + gaussian_yaw(gen);
		this->particles.at(p).x = nx;
		this->particles.at(p).y = ny;
		this->particles.at(p).theta = ntheta;
	}
}

float ParticleFilter::obsAssociationWeight(const Map &map_landmarks,
				LandmarkObs &obs,  double std_landmark[]){
	/*
		This method search for the closest landmark from the given observation.
		Once the closest landmark is found, the associated weight is compute.
	*/
	float dist;
	float closest_id = -1;
	float closest_dist = 0;

	// Searching for the closest landmark (Nearest-neighbors)
	bool in_range = false;
	for (size_t l = 0; l < map_landmarks.landmark_list.size(); l++) {
		dist = sqrt(pow(map_landmarks.landmark_list.at(l).x_f - obs.x, 2) + pow(map_landmarks.landmark_list.at(l).y_f - obs.y, 2));
		if (closest_id == -1 || dist < closest_dist){
			closest_id = l;
			closest_dist = dist;
		}
	}

	float weight;
	float sig_x = std_landmark[0];
	float sig_y = std_landmark[1];
	float x_obs = obs.x;
	float y_obs = obs.y;
	float mu_x = map_landmarks.landmark_list.at(closest_id).x_f;
	float mu_y = map_landmarks.landmark_list.at(closest_id).y_f;

	// Normalization term
	// 1 / (2.π.σx.σy)
	float gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
	// Exponent term
	// [(x - μx)**2) / 2σx**2] + [(y - μy)**2) / 2σy**2]
	float exponent = (pow(x_obs - mu_x, 2))/(2 * pow(sig_x, 2)) + (pow(y_obs - mu_y, 2))/(2 * pow(sig_y, 2));
	// Weight of this observation
	weight = gauss_norm * exp(-exponent);

	return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	/*
		Update the weights of each particle using a mult-variate Gaussian distribution.
		The observations are given in the VEHICLE'S coordinate system. Then, each
		observations is translate to the MAP's coordinate system.
	*/
	// Keep track of the total sum of all weights
	float total_weight_sum = 0;

	for (size_t p = 0; p < this->num_particles; p++) {
		double part_x = this->particles.at(p).x;
		double part_y = this->particles.at(p).y;
		double theta = this->particles.at(p).theta;
		// Init the current weight of this particles
		double total_weight = 1.;
		// Go through all observations, convert the coordinate
		// of each observation to map coordinate, then compute the weight
		// of this observation (if in the range of the sensor)
		for (size_t l = 0; l < observations.size(); l++) {
			double obs_x = observations.at(l).x;
			double obs_y = observations.at(l).y;
			// transform to map x coordinate
			double map_x = part_x + (cos(theta) * obs_x) - (sin(theta) * obs_y);
			// transform to map y coordinate
			double map_y = part_y + (sin(theta) * obs_x) + (cos(theta) * obs_y);
			// Compute the distance between the particle and this observations
			double dist = sqrt(pow(part_x - map_x, 2) + pow(part_y - map_y, 2));
			if (dist <= sensor_range){
				LandmarkObs obs;
	    		obs.x = map_x;
				obs.y = map_y;
				total_weight *= this->obsAssociationWeight(map_landmarks, obs, std_landmark);
			}
		}
		// Set the new weight of this particle
		this->particles.at(p).weight = total_weight;
		total_weight_sum += this->particles.at(p).weight;
	}
	this->total_weight_sum = total_weight_sum;
}

void ParticleFilter::resample() {
	/*
		Resample particles with replacement with probability proportional to their weight.
		The resample is done using the wheel method
	*/
	double max_weight = 0;


	for (size_t p = 0; p < this->num_particles; p++) {
		this->particles.at(p).weight /= this->total_weight_sum;
		if (this->particles.at(p).weight > max_weight){
			max_weight = this->particles.at(p).weight;
		}
	}
	// Sample a random index
	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(0, this->num_particles);
	int index = distribution(generator);
	float beta = 0.0;
	// List with the next new particles
	std::vector<Particle> n_particles;
	// Use to sample numbers between 0 and 1
	std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);
	for (size_t p = 0; p < this->num_particles; p++) {
		beta +=  dist(e2) * 2 * max_weight;
		while (beta > this->particles.at(index).weight){
			beta -= this->particles.at(index).weight;
			index = (index + 1) % this->num_particles;
		}
		// Att the new particle
		n_particles.push_back(this->particles.at(index));
	}
	this->particles = n_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
