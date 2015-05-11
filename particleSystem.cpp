#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <map>

#define IX(i,j,k) ((i)*(N+2)*(N+2)+(j)*(N+2)+(k))
#define SWAP(a,b,tmp) {tmp = a; a = b; b = tmp;}
#define FOR_EACH_GRID(N,i,j,k)	\
		for(i = 1; i <= N; i++)	\
for(j = 1; j <= N; j++) 	\
for(k = 1; k <= N; k++)

#include <FL/gl.h>

using namespace std;

static float prevT;
float ParticleSystem::particleRadius = 0.15;
Vec3f ParticleSystem::gravity = Vec3f(0, -9.8f, 0);
float ParticleSystem::airResistance = 0.5;
int ParticleSystem::particleNum = 500;
int ParticleSystem::particleReal = 0;
float ParticleSystem::spring_K = 500;
int ParticleSystem::particleNum_ponyTail = 8;
int ParticleSystem::particleNum_cloth_row = 15;
int ParticleSystem::particleNum_cloth_col = 5;
float ParticleSystem::deltaX = 0;
bool ParticleSystem::pipe = false;
bool ParticleSystem::pony = false;
bool ParticleSystem::cloth = false;
bool ParticleSystem::bounceOff = false;

float ParticleSystem::offset = 0;

Vec4f ParticleSystem::particleOrigin_pony = Vec4f(0,0,0,1);
Vec4f ParticleSystem::particleOrigin_cloth = Vec4f(0,0,0,1);

Vec4f ParticleSystem::pelvisPosition = Vec4f(0,0,0,1);
Vec4f ParticleSystem::footPosition = Vec4f(0,0,0,1);

Vec4f ParticleSystem::cloth_start = Vec4f(0,0,0,1);
Vec4f ParticleSystem::cloth_end = Vec4f(0,0,0,1);

Vec4f ParticleSystem::particleOrigin_pipe = Vec4f(0,0,0,1);


Vec3f ori_position = Vec3f(0,0,0);

float ParticleSystem::spring_cloth_K = 100;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
		// TODO
		time = 0;
		time2 = 0;
}


/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
		// TODO


}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
		// TODO

		time  = 0;
		time2 = 0;
		// pony tail

		if (pony) {

				particleReal = 0;
				ori_position = Vec3f(particleOrigin);

				ponyTail_particles.resize(particleNum_ponyTail, NULL);
				Particle* p = new Particle();
				p->position = Vec3f(particleOrigin_pony);
				p->velocity = Vec3f(0,0,0);
				p->force = Vec3f(0,0,0);
				p->mass = 1.0;
				ponyTail_particles[0] = p;


				for (int i = 1; i < particleNum_ponyTail; i++) {

						Particle* p = new Particle();
						p->position = Vec3f(particleOrigin_pony[0] + i * 0.5, particleOrigin_pony[1] + i * 0.1, particleOrigin_pony[2] + i * 0.1);
						p->velocity = Vec3f(0.15,0.1,0);
						p->force = Vec3f(gravity);
						p->mass = 1.0;
						ponyTail_particles[i] = p;
				}

		}

		// cloth

		if (cloth) {

				//  std::cout <<"start simulate " << t << std::endl;
				cloth_particles = new Particle*[particleNum_cloth_row];
				for (int i = 0; i < particleNum_cloth_row; i++) {
						cloth_particles[i] = new Particle[particleNum_cloth_col];
				}


				deltaX = (cloth_start[0] - cloth_end[0]) * 1.0 / 4.0;
				float deltaY = (cloth_start[1] - cloth_end[1]) * 1.0 / 4.0;
				float deltaZ = (cloth_start[2] - cloth_end[2]) * 1.0 / 4.0;


				for (int j = 0; j < particleNum_cloth_col; j++) {
						cloth_particles[0][j].position = Vec3f( cloth_start[0] - deltaX * j, cloth_start[1] - deltaY * j + 0.3, cloth_start[2] - deltaZ * j);
						cloth_particles[0][j].velocity = Vec3f(0,0,0);
						cloth_particles[0][j].force = Vec3f(gravity);
						cloth_particles[0][j].mass = 1.0;
				}


				for (int i = 1; i < particleNum_cloth_row; i++) {
						for (int j = 0; j < particleNum_cloth_col; j++) {

								cloth_particles[i][j].position = Vec3f(cloth_particles[0][0].position[0] - deltaX * j, cloth_particles[0][0].position[1] - 0.2* i, cloth_particles[0][0].position[2] + 0.15 * i);

								cloth_particles[i][j].force = Vec3f(gravity);
								cloth_particles[i][j].velocity = Vec3f(0,0,0);
								cloth_particles[i][j].mass = 1.0;

						}

				}

				//  std::cout <<"position: " << cloth_particles[0][0].position[0] <<"," << cloth_particles[0][0].position[1] << "," << cloth_particles[0][0].position[2] << std::endl;
				//  std::cout <<"position: " << cloth_particles[9][4].position[0] <<"," << cloth_particles[9][4].position[1] << "," << cloth_particles[9][4].position[2] << std::endl;
		}

		// These values are used by the UI ...
		// negative bake_end_time indicates that simulation
		// is still progressing, and allows the
		// indicator window above the time slider
		// to correctly show the "baked" region
		// in grey.
		bake_end_time = -1;
		simulate = true;
		dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
		// TODO

		particleReal = 0;
		particles.clear();
		if (pony) ponyTail_particles.clear();

		if (cloth) {
				for (int i = 0; i < particleNum_cloth_row; i++) delete[] cloth_particles[i];
				delete[] cloth_particles;
		}

		// These values are used by the UI
		simulate = false;
		dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
		// TODO

		particleReal = 0;
		particles.clear();
		if (pony) ponyTail_particles.clear();

		if (cloth) {
				for (int i = 0; i < particleNum_cloth_row; i++) delete[] cloth_particles[i];
				delete[] cloth_particles;
		}

		// These values are used by the UI
		simulate = false;
		dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
		// TODO

		//  std::cout <<"time: " << t << std::endl;
		// TODO
		if(t- prevT > 0.03){
				float deltaT = t - prevT;
				ss.update_fluid(deltaT);
				es.update(deltaT);
				if( t - prevT > .04 )
						printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
				prevT = t;
		}
		return;

		if (isSimulate()) {


				if (t - time > 0.3) {

						time = t;


						//  if (bounceOff) {

						//   particleNum = 5;
						//  }



						if  (particles.size() < particleNum) {

								Particle* p = new Particle();
								p->position = Vec3f(particleOrigin);
								p->velocity = Vec3f(-1, -2, 0);
								p->force = Vec3f(gravity);
								particles.push_back(p);
								particleReal++;
						}
				}

				float deltaT = t - prevT;

				typedef vector<Particle*>::const_iterator iter;

				for(iter i = particles.begin(); i != particles.end(); ++i) {

						// F = 1/2 * C * S * V^2
						Vec3f airResistance_cur = Vec3f(0,0,0);

						airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
						airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
						airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];

						if ((*i)->velocity[0] > 0) {

								(*i)->force[0] -= airResistance_cur[0];
						} else {

								(*i)->force[0] += airResistance_cur[0];
						}

						if ((*i)->velocity[1] > 0) {

								(*i)->force[1] -= airResistance_cur[1];
						} else {

								(*i)->force[1] += airResistance_cur[1];
						}

						if ((*i)->velocity[2] > 0) {

								(*i)->force[2] -= airResistance_cur[2];
						} else {

								(*i)->force[2] += airResistance_cur[2];
						}


						(*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
						(*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
						(*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;

						(*i)->position[0] += (*i)->velocity[0] * deltaT;
						(*i)->position[1] += (*i)->velocity[1] * deltaT;
						(*i)->position[2] += (*i)->velocity[2] * deltaT;


						if(bounceOff) {

								if ((*i)->position[1] < 0) {

										float len = ((*i)->velocity).length();

										Vec3f N = Vec3f(0,1,0);
										Vec3f V = -1 * Vec3f((*i)->velocity);
										V.normalize();


										double NVv = N * V;
										Vec3f R = N * 2 * NVv - V;

										R.normalize();


										(*i)->velocity[0] = len * R[0];
										(*i)->velocity[1] = len * R[1];
										(*i)->velocity[2] = len * R[2];


								}

						}


				}

				if (pipe) {

						pipe_computeForcesAndUpdateParticles(t);
				}

				if (pony) {
						ponyTail_computeForcesAndUpdateParticles(t);
				}

				if (cloth) {
						cloth_computeForcesAndUpdateParticles(t);
				}

				bakeParticles(t);

		}

		// Debugging info
		if( t - prevT > .04 )
				printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
		prevT = t;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
		// TODO
		es.draw();
		return;


		// state update

		if (isSimulate()) {

				typedef vector<Particle*>::const_iterator iter;


				if (!bounceOff) {


						for (iter i = particles_pipe.begin(); i != particles_pipe.end(); i++) {

								if ((*i) -> position[1] < 0 || (*i) -> position[0] > 5 || (*i) -> position[1] > 10) {


								}
						}



						for (iter i = particles.begin(); i != particles.end(); i++) {

								if ((*i)->position[1] < 0) {

										if ((*i) -> position[1] < 0) {

												particleReal--;
										}
								}
						}

				}

				// particles


				for(iter i = particles.begin(); i != particles.end(); ++i) {



						setDiffuseColor( rand() % 100 * 1.0/ 100 , rand() % 100 * 1.0/ 100, rand() % 100 * 1.0/ 100 );
						setAmbientColor( 1, 1, 1 );

						glPushMatrix();
						glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
						drawSphere(particleRadius);
						glPopMatrix();
				}

				if (pipe) {


						setDiffuseColor(176.0/256,196.0/256,222.0/256);
						setAmbientColor(0,0,0);

						for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {

								glPushMatrix();
								glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
								drawSphere(0.1);
								glPopMatrix();
						}
				}


				//ponyTail

				if (pony) {


						setDiffuseColor( 255.0 / 255 , 255.0 / 255, 240.0 / 255 );
						setAmbientColor (85.0/ 256,107.0 / 256,47.0 / 256);

						for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {


								glPushMatrix();
								glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
								drawSphere(particleRadius);
								glPopMatrix();


								if (i != ponyTail_particles.begin()) {

										glLineWidth(2.5);
										glColor3f(0.0, 0.0, 1.0);
										glBegin(GL_LINES);
										glVertex3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
										glVertex3f((*(i-1))->position[0], (*(i-1))->position[1], (*(i-1))->position[2]);
										glEnd();
								}

						}
				}

				// cloth


				if (cloth) {

						for (int i = 0; i < particleNum_cloth_row - 1; i++)
						{
								for (int j = 0; j < particleNum_cloth_col - 1; j++)
								{
										Vec3f p;

										p = cloth_particles[i][j].position;

										setDiffuseColor( 143.0 / 255 , 188.0 / 255, 143.0 / 255 );
										setAmbientColor (0,0,0);

										glPushMatrix();
										glTranslated(p[0], p[1], p[2]);
										drawSphere(0.05);
										glPopMatrix();

										glBegin(GL_QUADS);
										glVertex3f(p[0], p[1], p[2]);
										p = cloth_particles[i][j + 1].position;
										glVertex3f(p[0], p[1], p[2]);
										p = cloth_particles[i + 1][j + 1].position;
										glVertex3f(p[0], p[1], p[2]);
										p = cloth_particles[i + 1][j ].position;
										glVertex3f(p[0], p[1], p[2]);
										glEnd();

								}
						}

				}

		} else {

				std::vector<Vec3f> current_particles_position;
				std::map<float, std::vector<Vec3f> >::iterator result = bake_particles.find(t);
				if (result != bake_particles.end())
						current_particles_position = (*result).second;

				typedef vector<Vec3f>::const_iterator iter;

				for(iter i = current_particles_position.begin(); i != current_particles_position.end(); ++i) {

						setDiffuseColor( rand() % 100 * 1.0/ 100 , rand() % 100 * 1.0/ 100, rand() % 100 * 1.0/ 100 );
						setAmbientColor( 1, 1, 1 );
						glPushMatrix();
						glTranslated((*i)[0], (*i)[1], (*i)[2]);
						drawSphere(particleRadius);
						glPopMatrix();

				}




				if (pipe) {

						std::vector<Vec3f> current_particlesPipe_position;
						std::map<float, std::vector<Vec3f> >::iterator result = bake_particlesPipe.find(t);
						if (result != bake_particlesPipe.end())
								current_particlesPipe_position = (*result).second;

						typedef vector<Vec3f>::const_iterator iter;

						for(iter i = current_particlesPipe_position.begin(); i != current_particlesPipe_position.end(); ++i) {

								setDiffuseColor(176.0/256,196.0/256,222.0/256);
								setAmbientColor(0,0,0);
								glPushMatrix();
								glTranslated((*i)[0], (*i)[1], (*i)[2]);
								drawSphere(0.1);
								glPopMatrix();

						}

				}



				if (pony) {

						std::vector<Vec3f> current_particlesPony_position;
						std::map<float, std::vector<Vec3f> >::iterator result2 = bake_particlesPony.find(t);
						if (result2 != bake_particlesPony.end())
								current_particlesPony_position = (*result2).second;

						typedef vector<Vec3f>::const_iterator iter;


						for(iter i = current_particlesPony_position.begin(); i != current_particlesPony_position.end(); ++i) {

								setDiffuseColor( 255.0 / 255 , 255.0 / 255, 240.0 / 255 );
								setAmbientColor (85.0/ 256,107.0 / 256,47.0 / 256);

								glPushMatrix();
								glTranslated((*i)[0], (*i)[1], (*i)[2]);
								drawSphere(particleRadius);
								glPopMatrix();


								if (i != current_particlesPony_position.begin()) {

										glLineWidth(2.5);
										glColor3f(0.0, 0.0, 1.0);
										glBegin(GL_LINES);
										glVertex3f((*i)[0], (*i)[1], (*i)[2]);
										glVertex3f((*(i-1))[0], (*(i-1))[1], (*(i-1))[2]);
										glEnd();
								}
						}
				}


				if (cloth) {


						int index = -1;

						std::vector<Vec3f> current_particlesCloth_position;
						std::map<float, std::vector<Vec3f> >::iterator result3 = bake_particlesCloth.find(t);
						if (result3 != bake_particlesCloth.end())
								current_particlesCloth_position = (*result3).second;

						typedef vector<Vec3f>::const_iterator iter;

						for(iter i = current_particlesCloth_position.begin(); i != current_particlesCloth_position.end(); ++i) {



								index++;

								//std::cout <<"index: " << index << std::endl;

								setDiffuseColor( 143.0 / 255 , 188.0 / 255, 143.0 / 255 );
								setAmbientColor (0,0,0);

								glPushMatrix();
								glTranslated((*i)[0], (*i)[1], (*i)[2]);
								drawSphere(0.05);
								glPopMatrix();


								int row = index / particleNum_cloth_col;
								int col = index % particleNum_cloth_col;


								if (row != particleNum_cloth_row - 1 && col != particleNum_cloth_col - 1) {

										Vec3f p;

										glBegin(GL_QUADS);
										glVertex3f(current_particlesCloth_position[row * particleNum_cloth_col + col][0], current_particlesCloth_position[row * particleNum_cloth_col + col][1], current_particlesCloth_position[row * particleNum_cloth_col + col][2]);

										glVertex3f(current_particlesCloth_position[row * particleNum_cloth_col + col + 1][0], current_particlesCloth_position[row * particleNum_cloth_col + col + 1][1], current_particlesCloth_position[row * particleNum_cloth_col + col + 1][2]);

										glVertex3f(current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][0], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][1], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col + 1][2]);


										glVertex3f(current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][0], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][1], current_particlesCloth_position[(row + 1) * particleNum_cloth_col + col][2]);
										glEnd();


								}

						}

				}

		}

}



/** Adds the current configuration of particles to
 * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
		// TODO

		std::vector<Vec3f> current_particles_position;
		typedef vector<Particle*>::const_iterator iter;

		// state update

		for(iter i = particles.begin(); i != particles.end(); ++i) {

				Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);

				current_particles_position.push_back(position);

		}

		bake_particles.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particles_position));


		if (pipe) {


				std::vector<Vec3f> current_particlesPipe_position;
				typedef vector<Particle*>::const_iterator iter;

				// state update

				for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {

						Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);

						current_particlesPipe_position.push_back(position);

				}

				bake_particlesPipe.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesPipe_position));



		}


		if (pony) {

				std::vector<Vec3f> current_particlesPony_position;

				for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {

						Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);

						current_particlesPony_position.push_back(position);

				}

				bake_particlesPony.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesPony_position));

		}

		if (pony) {

				std::vector<Vec3f> current_particlesCloth_position;

				for(int i = 0; i < particleNum_cloth_row; i++) {
						for (int j = 0; j < particleNum_cloth_col; j++) {

								Vec3f position = Vec3f(cloth_particles[i][j].position[0], cloth_particles[i][j].position[1], cloth_particles[i][j].position[2]);
								current_particlesCloth_position.push_back(position);
						}
				}

				bake_particlesCloth.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesCloth_position));

		}
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
		// TODO

		bake_particles.clear();
		if (pony) bake_particlesPony.clear();
		if (cloth) bake_particlesCloth.clear();

}


void ParticleSystem::ponyTail_computeForcesAndUpdateParticles(float t)
{
		// TODO

		if (isSimulate()) {

				float dx = 0;
				float dy = 0;
				float dz = 0;

				float deltaT = t - prevT;

				typedef vector<Particle*>::const_iterator iter;

				iter i = ponyTail_particles.begin();


				if ((*i)->position[0] != particleOrigin_pony[0] ||  (*i)->position[1] != particleOrigin_pony[1] || (*i)->position[2] != particleOrigin_pony[2] ) {

						(*i)->position = Vec3f(particleOrigin_pony);
						dx = particleOrigin_pony[0] - (*i)->position[0];
						dy = particleOrigin_pony[1] - (*i)->position[1];
						dz = particleOrigin_pony[2] - (*i)->position[2];

				}


				int index = 0;

				for(iter i = ponyTail_particles.begin() + 1; i != ponyTail_particles.end(); ++i) {

						index++;

						// F = 1/2 * C * S * V^2
						Vec3f airResistance_cur = Vec3f(0,0,0);
						airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
						airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
						airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];

						if ((*i)->velocity[0] > 0) {

								(*i)->force[0] -= airResistance_cur[0];
						} else {

								(*i)->force[0] += airResistance_cur[0];
						}

						if ((*i)->velocity[1] > 0) {

								(*i)->force[1] -= airResistance_cur[1];
						} else {

								(*i)->force[1] += airResistance_cur[1];
						}

						if ((*i)->velocity[2] > 0) {

								(*i)->force[2] -= airResistance_cur[2];
						} else {

								(*i)->force[2] += airResistance_cur[2];
						}




						if (index < particleNum_ponyTail - 1) {


								Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);

								float distance_left = left.length();
								left.normalize();
								Vec3f force_left = spring_K * (1 - distance_left) * left;
								Vec3f right = Vec3f((*i)->position[0] - (*(i+1))->position[0], (*i)->position[1] - (*(i+1))->position[1], (*i)->position[2] - (*(i+1))->position[2]);
								float distance_right = right.length();
								right.normalize();
								Vec3f force_right = spring_K * (1 - distance_right) * right;

								(*i)->force[0] += force_left[0];
								(*i)->force[0] += force_right[0];

								(*i)->force[1] += force_left[1];
								(*i)->force[1] += force_right[1];

								(*i)->force[2] += force_left[2];
								(*i)->force[2] += force_right[2];

						} else {


								Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);
								float distance_left = left.length();
								left.normalize();
								Vec3f force_left = spring_K * (1 - distance_left) * left;

								(*i)->force[0]  +=  force_left[0];
								(*i)->force[1]  +=  force_left[1];
								(*i)->force[2]  +=  force_left[2];
						}


						if ((*i)->force.length() > 10) {
								(*i)->force.normalize();
								(*i)->force *= 10;
						}

						(*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.05;
						(*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.05;
						(*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.05;

						(*i)->position[0] += (*i)->velocity[0] * deltaT;
						(*i)->position[1] += (*i)->velocity[1] * deltaT;
						(*i)->position[2] += (*i)->velocity[2] * deltaT;


						(*i)->position[0] += dx;
						(*i)->position[1] += dy;
						(*i)->position[2] += dz;
				}
		}
}



void ParticleSystem::cloth_computeForcesAndUpdateParticles(float t)
{
		// TODO

		if (isSimulate()) {

				float deltaT = t - prevT;

				for (int i = 1; i < particleNum_cloth_row; i++) {
						for (int j = 0; j < particleNum_cloth_col; j++) {

								// F = 1/2 * C * S * V^2
								Vec3f airResistance_cur = Vec3f(0,0,0);
								airResistance_cur[0] = 0.5 * airResistance * 0.054 * cloth_particles[i][j].velocity[0] * cloth_particles[i][j].velocity[0];
								airResistance_cur[1] = 0.5 * airResistance * 0.054 * cloth_particles[i][j].velocity[1] * cloth_particles[i][j].velocity[1];
								airResistance_cur[2] = 0.5 * airResistance * 0.054* cloth_particles[i][j].velocity[2] * cloth_particles[i][j].velocity[2];

								if (cloth_particles[i][j].velocity[0] > 0) {

										cloth_particles[i][j].velocity[0] -= airResistance_cur[0];
								} else {

										cloth_particles[i][j].velocity[0] += airResistance_cur[0];
								}

								if (cloth_particles[i][j].velocity[1] > 0) {

										cloth_particles[i][j].force[1] -= airResistance_cur[1];
								} else {

										cloth_particles[i][j].force[1] += airResistance_cur[1];
								}

								if (cloth_particles[i][j].velocity[2] > 0) {

										cloth_particles[i][j].force[2] -= airResistance_cur[2];
								} else {

										cloth_particles[i][j].force[2] += airResistance_cur[2];
								}

								Vec3f force_left = Vec3f(0,0,0);
								Vec3f force_right = Vec3f(0,0,0);
								Vec3f force_up = Vec3f(0,0,0);
								Vec3f force_down = Vec3f(0,0,0);


								if (j != 0) {

										Vec3f left = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i][j - 1].position[0], cloth_particles[i][j].position[1] - cloth_particles[i][j - 1].position[1], cloth_particles[i][j].position[2] - cloth_particles[i][j - 1].position[2]);

										float distance_left = left.length();
										left.normalize();
										force_left = spring_cloth_K * (deltaX - distance_left) * left;

								}


								if (j != particleNum_cloth_col - 1) {

										Vec3f right = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i][j + 1].position[0], cloth_particles[i][j].position[1] - cloth_particles[i][j+1].position[1], cloth_particles[i][j].position[2] - cloth_particles[i][j + 1].position[2]);
										float distance_right = right.length();
										right.normalize();
										force_right = spring_cloth_K * (deltaX - distance_right) * right;
								}

								if (i != 0) {

										Vec3f up = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i - 1 ][j].position[0], cloth_particles[i][j].position[1] - cloth_particles[i - 1][j].position[1], cloth_particles[i][j].position[2] - cloth_particles[i - 1][j].position[2]);

										float distance_up = up.length();
										up.normalize();
										force_up = spring_cloth_K * (0.15 - distance_up) * up;

								}


								if (i != particleNum_cloth_row - 1) {

										Vec3f down = Vec3f(cloth_particles[i][j].position[0] - cloth_particles[i + 1 ][j].position[0], cloth_particles[i][j].position[1] - cloth_particles[i + 1][j].position[1], cloth_particles[i][j].position[2] - cloth_particles[i + 1][j].position[2]);

										float distance_down = down.length();
										down.normalize();
										force_down = spring_cloth_K * (0.15 - distance_down) * down;
								}

								cloth_particles[i][j].force[0] += force_left[0];
								cloth_particles[i][j].force[0] += force_right[0];
								cloth_particles[i][j].force[0] += force_up[0];
								cloth_particles[i][j].force[0] += force_down[0];

								cloth_particles[i][j].force[1] += force_left[1];
								cloth_particles[i][j].force[1] += force_right[1];
								cloth_particles[i][j].force[1] += force_up[1];
								cloth_particles[i][j].force[1] += force_down[1];
								cloth_particles[i][j].force[1] += 0.001;

								cloth_particles[i][j].force[2] += force_left[2];
								cloth_particles[i][j].force[2] += force_right[2];
								cloth_particles[i][j].force[2] += force_up[2];
								cloth_particles[i][j].force[2] += force_down[2];
								cloth_particles[i][j].force[2] += 0.8;


								if (cloth_particles[i][j].force.length() > 10) {
										cloth_particles[i][j].force.normalize();
										cloth_particles[i][j].force *= 10;
								}


								cloth_particles[i][j].velocity[0] += cloth_particles[i][j].force[0] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;
								cloth_particles[i][j].velocity[1] += cloth_particles[i][j].force[1] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;
								cloth_particles[i][j].velocity[2] += cloth_particles[i][j].force[2] * 1.0 / cloth_particles[i][j].mass * deltaT * 0.01;

								cloth_particles[i][j].position[0] += cloth_particles[i][j].velocity[0] * deltaT;
								cloth_particles[i][j].position[1] += cloth_particles[i][j].velocity[1] * deltaT;
								cloth_particles[i][j].position[2] += cloth_particles[i][j].velocity[2] * deltaT;

						}

				}
		}
}


void ParticleSystem::pipe_computeForcesAndUpdateParticles(float t){

		if (isSimulate()) {


				if (t - time2 > 0.05) {

						time2 = t;

						if  (particles_pipe.size() < particleNum) {

								Particle* p = new Particle();
								p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
								p->velocity = Vec3f(4, 6, 0);
								p->force = Vec3f(gravity);
								particles_pipe.push_back(p);


								p = new Particle();
								p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
								p->velocity = Vec3f(4, 6, 0);
								p->force = Vec3f(gravity);
								particles_pipe.push_back(p);


								p = new Particle();
								p->position = Vec3f(particleOrigin_pipe[0]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[1]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1, particleOrigin_pipe[2]+ (rand() % 100 * 1.0/ 50 - 1) * 0.1);
								p->velocity = Vec3f(4, 6, 0);
								p->force = Vec3f(gravity);
								particles_pipe.push_back(p);

						}
				}

				float deltaT = t - prevT;

				typedef vector<Particle*>::const_iterator iter;

				for(iter i = particles_pipe.begin(); i != particles_pipe.end(); ++i) {

						// F = 1/2 * C * S * V^2
						Vec3f airResistance_cur = Vec3f(0,0,0);

						airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
						airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
						airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];

						if ((*i)->velocity[0] > 0) {

								(*i)->force[0] -= airResistance_cur[0];
						} else {

								(*i)->force[0] += airResistance_cur[0];
						}

						if ((*i)->velocity[1] > 0) {

								(*i)->force[1] -= airResistance_cur[1];
						} else {

								(*i)->force[1] += airResistance_cur[1];
						}

						if ((*i)->velocity[2] > 0) {

								(*i)->force[2] -= airResistance_cur[2];
						} else {

								(*i)->force[2] += airResistance_cur[2];
						}


						(*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
						(*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
						(*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;

						(*i)->position[0] += (*i)->velocity[0] * deltaT;
						(*i)->position[1] += (*i)->velocity[1] * deltaT;
						(*i)->position[2] += (*i)->velocity[2] * deltaT;
				}
		}
}

void FluidSystem::draw_fluid(void){
		int i, j,k;
		double x, y , z, h;
		double d[8];

		h = 5.0f/N;
		//	glDisable(GL_LIGHTING);
		glPushMatrix();

		glTranslatef(position[0] - 7.5,position[1]-0.5 -5.0/2, position[2] - 2.5);
		//	cout<< position<<endl;
		glBegin ( GL_QUADS );

		for ( i=1 ; i<N-1 ; i++ ) {
				x = (i-0.5f)*h;
				for ( j=1 ; j<N-1 ; j++ ) {
						y = (j-0.5f)*h;
						for(k = 1; k < N-1; k++){
								z = (k -0.5f)*h;
								d[0] = dens[IX(i,j,k)];
								d[1] = dens[IX(i,j,k-1)];
								d[2] = dens[IX(i,j+1,k-1)];
								d[3] = dens[IX(i,j+1,k)];
								d[4] = dens[IX(i-1,j,k)];
								d[5] = dens[IX(i-1,j,k-1)];
								d[6] = dens[IX(i-1,j+1,k-1)];
								d[7] = dens[IX(i-1,j+1,k)];
								int t;
								float opa = 0.4;
								bool pass = true;
								setEmissionColor(0.0,0.0,0.0);
								for(t=0;t<8;t++){
										if(d[t] < 0.05){
												pass = false;
												break;
										}
										if(d[t] > 2){
												setEmissionColor(0.8, 0.2, 0.05);
												//cout << t << ":" << d[t];
												opa = 0.99;
										}

								}
								if(!pass)
										continue;
								//							setLightPosition(-4,4,4, 1);
								//							setAmbientColor(0.7,0.3,0.0);
								setLightPosition(-1,0,0, 1);
								setAmbientColor(1,1.0,1.0);
								setSpecularColor(0.0,0.0,0.0);
								glScalef(2.0,2.0,2.0);

								glNormal3d( 1.0 ,0.0, 0.0);			// +x side
								setDiffuseColor(d[0],d[0],d[0],d[0]>opa?opa:d[0]);
								glVertex3d( x, y, z);
								setDiffuseColor(d[1],d[1],d[1],d[1]>opa?opa:d[1]);
								glVertex3d( x, y, z-h);
								setDiffuseColor(d[2],d[2],d[2],d[2]>opa?opa:d[2]);
								glVertex3d( x,  y+h,z-h);
								setDiffuseColor(d[3],d[3],d[3],d[3]>opa?opa:d[3]);
								glVertex3d( x,  y+h, z);

								glNormal3d( 0.0 ,0.0, -1.0);		// -z side
								setDiffuseColor(d[1],d[1],d[1],d[1]>opa?opa:d[1]);
								glVertex3d( x, y, z-h);
								setDiffuseColor(d[5],d[5],d[5],d[5]>opa?opa:d[5]);
								glVertex3d( x-h,y,z-h);
								setDiffuseColor(d[6],d[6],d[6],d[6]>opa?opa:d[6]);
								glVertex3d( x-h,  y+h,z-h);
								setDiffuseColor(d[2],d[2],d[2],d[2]>opa?opa:d[2]);
								glVertex3d( x,  y+h,z-h);

								glNormal3d(-1.0, 0.0, 0.0);			// -x side
								setDiffuseColor(d[5],d[5],d[5],d[5]>opa?opa:d[5]);
								glVertex3d(x-h,y,z-h);
								setDiffuseColor(d[4],d[4],d[4],d[4]>opa?opa:d[4]);
								glVertex3d(x-h,y, z);
								setDiffuseColor(d[7],d[7],d[7],d[7]>opa?opa:d[7]);
								glVertex3d(x-h, y+h , z);
								setDiffuseColor(d[6],d[6],d[6],d[6]>opa?opa:d[6]);
								glVertex3d(x-h, y+h,z-h);

								glNormal3d( 0.0, 0.0, 1.0);			// +z side
								setDiffuseColor(d[4],d[4],d[4],d[4]>opa?opa:d[4]);
								glVertex3d(x-h,y, z);
								setDiffuseColor(d[0],d[0],d[0],d[0]>opa?opa:d[0]);
								glVertex3d( x,y, z);
								setDiffuseColor(d[3],d[3],d[3],d[3]>opa?opa:d[3]);
								glVertex3d( x,  y+h, z);
								setDiffuseColor(d[7],d[7],d[7],d[7]>opa?opa:d[7]);
								glVertex3d(x-h,  y+h, z);

								glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
								setDiffuseColor(d[3],d[3],d[3],d[3]>opa?opa:d[3]);
								glVertex3d( x, y+ h, z);
								setDiffuseColor(d[2],d[2],d[2],d[2]>opa?opa:d[2]);
								glVertex3d( x, y+ h,z-h);
								setDiffuseColor(d[6],d[6],d[6],d[6]>opa?opa:d[6]);
								glVertex3d(x-h, y+ h,z-h);
								setDiffuseColor(d[7],d[7],d[7],d[7]>opa?opa:d[7]);
								glVertex3d(x-h, y+ h, z);

								glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
								setDiffuseColor(d[0],d[0],d[0],d[0]>opa?opa:d[0]);
								glVertex3d( x,y, z);
								setDiffuseColor(d[4],d[4],d[4],d[4]>opa?opa:d[4]);
								glVertex3d(x-h,y, z);
								setDiffuseColor(d[5],d[5],d[5],d[5]>opa?opa:d[5]);
								glVertex3d(x-h,y,z-h);
								setDiffuseColor(d[1],d[1],d[1],d[1]>opa?opa:d[1]);
								glVertex3d( x,y,z-h);
						}
				}
		}

		glEnd ();
		glPopMatrix();
		//glEnable(GL_LIGHTING);
}


void FluidSystem::add_to_array(double* x, int index, double quantity)
{
		x[index] += quantity;
}

void FluidSystem::gs_solver(int N, double* x, double *x0, double rate, double div)
{
		int i,j,k,iter;
		for(iter = 0; iter < iteration_count; iter++){
				FOR_EACH_GRID(N,i,j,k)
				{
						x[IX(i,j,k)] = (x0[IX(i,j,k)] +
										rate * (x[IX(i-1,j,k)] + x[IX(i+1,j,k)] +
												x[IX(i,j-1,k)] + x[IX(i,j+1,k)] +
												x[IX(i,j,k-1)] + x[IX(i,j,k+1)]
											   )
									   )/div;
				}

		}
}

void FluidSystem::project(int N, double *u, double *v, double *w, double *g, double *g0)
{
		int i,j,k;
		FOR_EACH_GRID(N,i,j,k)
		{
				g0[IX(i,j,k)]= -1.0/3.0 * (u[IX(i+1,j,k)] - u[IX(i-1,j,k)] +
								v[IX(i,j+1,k)] - v[IX(i,j-1,k)] +
								w[IX(i,j,k+1)] - w[IX(i,j,k-1)])/N;
				g[IX(i,j,k)] = 0;
		}
		/*
		   FOR_EACH_GRID(N,i,j,k)
		   {

		   cout << "g0" << i << "," << j << "," << k <<":"<< g0[IX(i,j,k)] << ";";
		   }
		   cout << endl;
		 */
		//use gs_solver to iteratively compute gradient
		gs_solver(N, g, g0, 1, 6);
		/*
		   FOR_EACH_GRID(N,i,j,k)
		   {

		   cout << "g" << i << "," << j << "," << k <<":"<< g[IX(i,j,k)] << ";";
		   }
		   cout << endl;
		 */
		FOR_EACH_GRID(N,i,j,k)
		{
				u[IX(i,j,k)] -= 0.5 * N * (g[IX(i+1,j,k)] - g[IX(i-1,j,k)]);
				v[IX(i,j,k)] -= 0.5 * N * (g[IX(i,j+1,k)] - g[IX(i,j-1,k)]);
				w[IX(i,j,k)] -= 0.5 * N * (g[IX(i,j,k+1)] - g[IX(i,j,k-1)]);
		}
}

//back trace
void FluidSystem::advect(int N, double *d, double *d0, double *u, double *v, double *w, double dt)
{
		int i,j,k;
		double x,y,z;
		int xi,yi,zi;
		double xw, yw, zw;
		double xwn, ywn, zwn;
		//backward trace

		//dt = dt * N;
		//dt = 1;
		dt = dt * N;
		//	cout << dt <<endl;
		FOR_EACH_GRID(N,i,j,k)
		{
				//which grid?
				x = i + dt*u[IX(i,j,k)];
				y = j + dt*v[IX(i,j,k)];
				z = k + dt*w[IX(i,j,k)];
				if(x <= 0 || y <= 0 || z <= 0 || x >= N || y >= N || z >= N){//outside the grid
						d[IX(i,j,k)] = 0.0;
						continue;
				}

				x = i - dt*u[IX(i,j,k)];
				y = j - dt*v[IX(i,j,k)];
				z = k - dt*w[IX(i,j,k)];

				//interpolate
				if(x <= 0 || y <= 0 || z <= 0 || x >= N || y >= N || z >= N){//outside the grid
						d[IX(i,j,k)] = 0.0;
						continue;
				}
				//		printf("%d %d %d\n", i,j,k);
				//		printf("%f\n", w[IX(i,j,k)]);
				//		printf("%f %f %f\n", x,y, z);
				xi = (int)x; yi = (int)y; zi = (int)z;
				xwn = x - xi; ywn = y - yi; zwn = z - zi;
				xw = 1.0 - xwn; yw = 1.0 - ywn;	zw = 1.0 - zwn;

				//		printf("%d %d %d\n", xi,yi, zi);
				d[IX(i,j,k)] =	xw * yw * zw * d0[IX(xi,yi,zi)] +
						xwn * yw * zw * d0[IX(xi+1,yi,zi)] +
						xw * ywn * zw * d0[IX(xi,yi+1,zi)] +
						xwn * ywn * zw * d0[IX(xi+1,yi+1,zi)] +
						xw * yw * zwn * d0[IX(xi,yi,zi+1)] +
						xwn * yw * zwn * d0[IX(xi+1,yi,zi+1)] +
						xw * ywn * zwn * d0[IX(xi,yi+1,zi+1)] +
						xwn * ywn * zwn * d0[IX(xi+1,yi+1,zi+1)];
				//			cout<<xwn <<","<< ywn  <<","<<zwn <<","<< d0[IX(xi+1,yi+1,zi+1)]<< endl;
				//		printf("%f\n\n",d[IX(i,j,k)]);
		}

		/*
		//extend world
		xi = (int)x; yi = (int)y; zi = (int)z;
		xi = xi < 1 ? 1 : xi > N? N: xi;
		xi = xi < 1 ? 1 : xi > N? N: xi;
		yi = yi < 1 ? 1 : yi > N? N: yi;
		zi = zi < 1 ? 1 : zi > N? N: zi;
		xwn = x - xi; ywn = y - yi; zwn = z - zi;
		xwn = (xwn > 1.0)||(xwn < 0.0) ? 0.0 : xwn;
		ywn = (ywn > 1.0)||(ywn < 0.0) ? 0.0 : ywn;
		zwn = (zwn > 1.0)||(zwn < 0.0) ? 0.0 : zwn;
		xw = 1.0 - xwn; yw = 1.0 - ywn;	zw = 1.0 - zwn;

		d[IX(i,j,k)] =	xw * yw * zw * d0[IX(xi,yi,zi)] +
		xwn * yw * zw * d0[IX(xi+1,yi,zi)] +
		xw * ywn * zw * d0[IX(xi,yi+1,zi)] +
		xwn * ywn * zw * d0[IX(xi+1,yi+1,zi)] +
		xw * yw * zwn * d0[IX(xi,yi,zi+1)] +
		xwn * yw * zwn * d0[IX(xi+1,yi,zi+1)] +
		xw * ywn * zwn * d0[IX(xi,yi+1,zi+1)] +
		xwn * ywn * zwn * d0[IX(xi+1,yi+1,zi+1)];
		}
		 */
}
void FluidSystem::diffuse(int N, double *d, double *d0, double rate)
{
		gs_solver(N, d, d0, rate, 1+6*rate);
}
void FluidSystem::dens_step(int N, double *d, double *d0, double *u, double *v, double *w, double dif, double dt)
{
		double *tmp;
		int i,j,k;
		SWAP(d0, d, tmp);
		diffuse(N, d, d0, dif);
		SWAP(d0, d, tmp);
		advect(N, d, d0, u, v, w, dt);
}
void FluidSystem::vel_step(int N, double *u, double *v, double *w, double *u0, double *v0, double *w0, double vis, double dt)
{
		double *tmp;
		int i,j,k;
		SWAP(u, u0, tmp); SWAP(v, v0, tmp);	SWAP(w, w0, tmp);
		diffuse(N, u, u0, vis);
		diffuse(N, v, v0, vis);
		diffuse(N, w, w0, vis);

		project(N, u, v, w, u0, v0);
		SWAP(u0, u, tmp);
		SWAP(v0, v, tmp);
		SWAP(w0, w, tmp);
		//		cout << "!!!!" << w[IX(21,22,12)]<<endl;
		advect(N, u, u0, u0, v0, w0, dt);
		//	cout << "!!!!" <<endl;
		advect(N, v, v0, u0, v0, w0, dt);
		advect(N, w, w0, u0, v0, w0, dt);
		/*
		   FOR_EACH_GRID(N,i,j,k)
		   {
		   if(u[IX(i,j,k)] != 0.0)
		   cout << "u:"<<i << "," << j << "," << k <<":"<< u[IX(i,j,k)] << ";" <<endl;
		   if(v[IX(i,j,k)] != 0.0)
		   cout << "v:"<<i << "," << j << "," << k <<":"<< v[IX(i,j,k)] << ";" <<endl;
		   if(w[IX(i,j,k)] != 0.0)
		   cout << "w:"<<i << "," << j << "," << k <<":"<< w[IX(i,j,k)] << ";" <<endl;
		   }
		 */
		project(N, u, v, w, u0, v0);
		/*
		   FOR_EACH_GRID(N,i,j,k)
		   {
		   if(u[IX(i,j,k)] > 0.1)
		   cout << "u:"<<i << "," << j << "," << k <<":"<< u[IX(i,j,k)] << ";" <<endl;
		   }
		 */
}

float FluidSystem::gradient_to_velocity(float neg, float pos){
		float dif = (pos - neg)/3.0;
		return dif;
}
void FluidSystem::update_fluid(double dt){
		double *tmp;
		int i,j,k;

		float inner = 10.0f;
		float inner14 = 7.0f;
		int jitterx = -1.0 + rand()%3;
		int jittery = rand()%3;
		int jitterz = -1.0 + rand()%3;

		add_to_array(dens, IX(N/2,1,N/2), 200.0);
		//add fluid and pressure
		//add_to_array(dens, IX(N/2+jitterx,1+jittery,N/2+jitterz), 200.0);
		/*
		   add_to_array(u,IX(N/2-1+jitterx,1+jittery,N/2+jitterz),inner);;
		   add_to_array(u,IX(N/2-1+jitterx,1+jittery,N/2-1+jitterz),inner14);;
		   add_to_array(u,IX(N/2-1+jitterx,1+jittery,N/2+1+jitterz),inner14);;

		   add_to_array(u,IX(N/2+1+jitterx,1+jittery,N/2+jitterz),-inner);;
		   add_to_array(u,IX(N/2+1+jitterx,1+jittery,N/2-1+jitterz),-inner14);;
		   add_to_array(u,IX(N/2+1+jitterx,1+jittery,N/2+1+jitterz),-inner14);;

		   add_to_array(w,IX(N/2+1+jitterx,1+jittery,N/2-1+jitterz),inner14);;
		   add_to_array(w,IX(N/2,1+jitterx+jittery,N/2-1+jitterz),inner);;
		   add_to_array(w,IX(N/2-1+jitterx,1+jittery,N/2-1+jitterz),inner14);;

		   add_to_array(w,IX(N/2+1+jitterx,1+jittery,N/2+1+jitterz),-inner14);;
		   add_to_array(w,IX(N/2+jitterx,1+jittery,N/2+1+jitterz),-inner);;
		   add_to_array(w,IX(N/2-1+jitterx,1+jittery,N/2+1+jitterz),-inner14);
		 */
		//disturbe
		double positive;
		double negative;
		FOR_EACH_GRID(N,i,j,k){
				if(dens[IX(i,j,k)] > 10.0){//heat
						u[IX(i,j,k)] += gradient_to_velocity(dens[IX(i-1,j,k)], dens[IX(i+1,j,k)]);
						u[IX(i,j,k)] += gradient_to_velocity(dens[IX(i-1,j,k-1)], dens[IX(i+1,j,k+1)])/1.414;
						u[IX(i,j,k)] += gradient_to_velocity(dens[IX(i-1,j,k+1)], dens[IX(i+1,j,k-1)])/1.414;
						v[IX(i,j,k)] += 3.0f;
						w[IX(i,j,k)] += gradient_to_velocity(dens[IX(i,j,k-1)], dens[IX(i,j,k+1)]);
						w[IX(i,j,k)] += gradient_to_velocity(dens[IX(i-1,j,k-1)], dens[IX(i+1,j,k+1)])/1.414;
						w[IX(i,j,k)] += gradient_to_velocity(dens[IX(i+1,j,k-1)], dens[IX(i-1,j,k+1)])/1.414;
				}
				/*
				   positive = rand()/(double)(RAND_MAX) / 4;
				   negative = rand()/(double)(RAND_MAX+1) /4;
				   add_to_array(u, IX(i,j,k), positive +negative);
				   positive = rand()/(double)(RAND_MAX) / 4;
				   negative = rand()/(double)(RAND_MAX+1) /4;
				   add_to_array(v, IX(i,j,k), positive+negative);
				   positive = rand()/(double)(RAND_MAX)/4;
				   negative = rand()/(double)(RAND_MAX+1)/4;
				   add_to_array(w, IX(i,j,k), positive+negative);
				 */
				//decay
				if(dens[IX(i,j,k)] > 0.1){
						dens[IX(i,j,k)] *= 0.985;
				}else
						dens[IX(i,j,k)] = 0;


				//u[IX(i,j,k)] *= 0.99;
				//v[IX(i,j,k)] *= 0.99;
				//w[IX(i,j,k)] *= 0.99;
		}

		vel_step ( N, u, v, w, u0, v0, w0, visc, dt);

		//cout << "!!" <<endl;

		//decay
		/*
		   FOR_EACH_GRID(N,i,j,k){
		//	u[IX(i,j,k)] *= 0.99;
		//	v[IX(i,j,k)] *= 0.99;
		//	w[IX(i,j,k)] *= 0.99;
		if(u[IX(i,j,k)] < 0.01){
		u[IX(i,j,k)] = 0;
		}
		if(v[IX(i,j,k)] < 0.01){
		v[IX(i,j,k)] = 0;
		}
		if(w[IX(i,j,k)] < 0.01){
		w[IX(i,j,k)] = 0;
		}
		//disturb
		if(dens[IX(i,j,k)] < 0.1){
		dens[IX(i,j,k)] = 0;
		}
		}
		 */
		//	SWAP(dens, dens0,tmp);
		dens_step (N, dens, dens0, u, v, w, diff, dt);

		/*
		   double test = 0;
		//smoke decay
		FOR_EACH_GRID(N,i,j,k)
		{

		if(dens[IX(i,j,k)] > 0.1){
		cout << i << ","<< j << "," << k <<":"<< dens[IX(i,j,k)] << ";";
		}
		//	if(dens[IX(i,j,k)] < 0.01){
		//			dens[IX(i,j,k)] = 0;
		//	}
		//	u[IX(i,j,k)] *= 0.8;
		//	v[IX(i,j,k)] *= 0.8;
		//	w[IX(i,j,k)] *= 0.8;
		//	test+=(u[IX(i,j,k)]);
		//	test+= (v[IX(i,j,k)]);
		//	test+= (w[IX(i,j,k)]);
		//test+= abs(u[IX(i,j,k)]);
		//test+= abs(v[IX(i,j,k)]);
		//test+= abs(w[IX(i,j,k)]);
		test+= dens[IX(i,j,k)];

		//	if(u[IX(i,j,k)] != 0.0){
		//		cout << i << ","<< j << "," << k <<":"<< u[IX(i,j,k)] << ";";
		//	}
		//	if(v[IX(i,j,k)] != 0.0){
		//			cout << i << ","<< j << "," << k <<":"<< v[IX(i,j,k)] << ";";
		//	}
		//	if(w[IX(i,j,k)] != 0.0){
		//			cout << i << ","<< j << "," << k <<":"<< w[IX(i,j,k)] << ";";
		//	}
		}
		cout << "Total:" << test << endl;
		 */
		//cout << u[IX(1,1,1)] << endl;
}

void ExplosionSystem::explode(Vec3f position){
		for(std::vector<Particle>::iterator it = emitters.begin(); it != emitters.end();it++){
				(*it).life = 4 + 2 * (rand() /(double)RAND_MAX);
				(*it).position = position;
				float x_r = -1 + 2 * (rand()/(double)RAND_MAX);
				float y_r = 2 + (rand()/(double)RAND_MAX);
				float z_r = -1 + 2 *( rand()/(double)RAND_MAX);
				(*it).velocity = Vec3f(x_r, y_r ,z_r);
				//	cout<< "V"<<(*it).velocity << endl;
		}
		start = true;
}
void ExplosionSystem::update(float dt){
		if(!start)
				explode(Vec3f(-4,4,4));
		if(start){
				for(std::vector<Particle>::iterator it = tails.begin(); it != tails.end();){
						(*it).life -= (dt);
						if(((*it).life) <= 0){
								it = tails.erase(it);
								continue;
						}
						it++;
				}

				for(std::vector<Particle>::iterator it = emitters.begin(); it != emitters.end();it++){
						(*it).velocity[1] += a * (dt);
						(*it).position = (*it).position + (*it).velocity * (dt);
						//cout << (*it).position <<endl;
						//cout << "dt"<<dt<<":"<< (*it).life;
						(*it).life -= (dt);
						(*it).counter += (dt);
						if(((*it).life) <= 0){
								(*it).life = 1 + 5 * (rand() /(double)RAND_MAX);
								(*it).position = Vec3f(-4,4,4);
								(*it).counter = 0;
								float x_r = -1 + 2 * (rand()/(double)RAND_MAX);
								float y_r = 2 + (rand()/(double)RAND_MAX);
								float z_r = -1 + 2 *( rand()/(double)RAND_MAX);
								(*it).velocity = Vec3f(x_r, y_r ,z_r);
								//			it = emitters.erase(it);
								//			continue;
						}

						//add tails
						if((*it).counter >= 0.1){
								(*it).counter = 0;
								Particle ps;
								ps.life = 1.0;
								ps.position = (*it).position;
								tails.push_back(ps);
						}
				}
		}
}
void ExplosionSystem::draw(){
		for(std::vector<Particle>::iterator it = emitters.begin(); it != emitters.end(); ++it){
				glPushMatrix();
				glTranslatef( (*it).position[0], (*it).position[1], (*it).position[2]);
				//setAmbientColor( (*it).color[0], 0,0);
				float temp = (*it).life / 6.5;
				temp *= temp;
				//setDiffuseColor(0.75 +temp*0.2, 4+temp*0.3, 0.05+temp*0.1);
				//setDiffuseColor(0.75, 0.3, 0.05);
				//setAmbientColor(1, 1, 1);
				//setSpecularColor(1,0,0);
				setLightPosition(-4,4,4, 1);
				setDiffuseColor((*it).life,(*it).life, (*it).life,(*it).life);
				setAmbientColor(0.7,0.2,0.0);
				setSpecularColor(0.2,0.0,0.0);
				setEmissionColor(0.75+temp*0.2,0.4+temp*0.3, 0.1 +temp *0.1);

				glScalef(0.1,0.1,0.1);
				glBegin( GL_QUADS );
				glNormal3d( 1.0 ,0.0, 0.0);			// +x side
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d( 0.25,0.0,-0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);
				glVertex3d( 0.25,  2*0.25, 0.25);

				glNormal3d( 0.0 ,0.0, -1.0);		// -z side
				glVertex3d( 0.25,0.0,-0.25);
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);

				glNormal3d(-1.0, 0.0, 0.0);			// -x side
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);

				glNormal3d( 0.0, 0.0, 1.0);			// +z side
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d( 0.25,  2*0.25, 0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);

				glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
				glVertex3d( 0.25,  2*0.25, 0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);

				glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d( 0.25,0.0,-0.25);
				glEnd();
				glPopMatrix();
		}

		for(std::vector<Particle>::iterator it = tails.begin(); it != tails.end(); ++it){
				//glDisable(GL_LIGHTING);
				glPushMatrix();
				glTranslatef( (*it).position[0], (*it).position[1], (*it).position[2]);
				//glColor4f((*it).life,(*it).life,(*it).life,(*it).life);
				//glColor4f((*it).life,0,0,(*it).life);
			setLightPosition(-4,4,4, 1);
				setDiffuseColor((*it).life,(*it).life, (*it).life,(*it).life);
				setAmbientColor(0.7,0.2,0.0);
				setSpecularColor(0.2,0.0,0.0);
				setEmissionColor(0,0,0);


				float size = 0.2*(1.1-(*it).life);
				glScalef(size,size,size);
				glBegin( GL_QUADS );
				glNormal3d( 1.0 ,0.0, 0.0);			// +x side
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d( 0.25,0.0,-0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);
				glVertex3d( 0.25,  2*0.25, 0.25);

				glNormal3d( 0.0 ,0.0, -1.0);		// -z side
				glVertex3d( 0.25,0.0,-0.25);
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);

				glNormal3d(-1.0, 0.0, 0.0);			// -x side
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);

				glNormal3d( 0.0, 0.0, 1.0);			// +z side
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d( 0.25,  2*0.25, 0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);

				glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
				glVertex3d( 0.25,  2*0.25, 0.25);
				glVertex3d( 0.25,  2*0.25,-0.25);
				glVertex3d(-0.25,  2*0.25,-0.25);
				glVertex3d(-0.25,  2*0.25, 0.25);

				glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
				glVertex3d( 0.25,0.0, 0.25);
				glVertex3d(-0.25,0.0, 0.25);
				glVertex3d(-0.25,0.0,-0.25);
				glVertex3d( 0.25,0.0,-0.25);
				glEnd();
				glPopMatrix();
				//glEnable(GL_LIGHTING);
		}
}
