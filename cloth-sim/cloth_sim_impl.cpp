#include "cloth_sim_impl.h"

ClothSimImpl::ClothSimImpl() : edu::rpi::cats::utilities::clothsim::ClothSimulator()
{
	this->numX = 0;
	this->numY = 0;
	this->n_points = 0;
	this->cloth_width = 0;
	this->cloth_length = 0;
	this->cloth_mass = 0;
	this->new_grasp_positions = false;
	this->grasp_positions_updated = false;
	this->image_width = 512;
	this->image_height = 424;

	this->dt = 1.f / 120.f;
}

ClothSimImpl::ClothSimImpl(uint16_t nX, uint16_t nY, btScalar width, btScalar length, btScalar mass, btScalar stiffness, btScalar bending_stiffness, btScalar damping) : edu::rpi::cats::utilities::clothsim::ClothSimulator()
{
	std::cout << "Generating object" << std::endl;
	this->numX = nX;
	this->numY = nY;
	this->n_points = ((uint32_t)nX)*((uint32_t)nY);
	this->cloth_width = width;
	this->cloth_length = length;
	this->cloth_mass = mass;
	this->cloth_stiffness = stiffness;
	this->cloth_bending_stiffness = bending_stiffness;
	this->cloth_damping = damping;
	this->new_grasp_positions = false;
	this->grasp_positions_updated = false;
	this->image_width = 512;
	this->image_height = 424;
	this->image_data = new float[this->image_width * this->image_height];
	this->ROk.setIdentity();
	this->K = btMatrix3x3(350.f, 0.f, 256.f, 0.f, 350.f, 212.f, 0.f, 0.f, 1.f);

	this->_recording = false;

	this->dt = 1.f / 120.f;

	if (initWorldAndCloth() > 0)
	{
		this->x = new double[this->cloth->m_nodes.size()];
		this->y = new double[this->cloth->m_nodes.size()];
		this->z = new double[this->cloth->m_nodes.size()];
		this->f = new double[this->cloth->m_nodes.size()];
		//this->fx = new double[this->cloth->m_nodes.size()];
		//this->fy = new double[this->cloth->m_nodes.size()];
		//this->fz = new double[this->cloth->m_nodes.size()];
		for (int i = 0, ni = this->cloth->m_nodes.size(); i < ni; i++)
			this->spring_forces.push_back(btVector3(0.f, 0.f, 0.f));

	}
	else
		std::cout << "Something went wrong and things aren't simulating" << std::endl;
}

ClothSimImpl::~ClothSimImpl()
{
	shutdown();
}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothDefinition > ClothSimImpl::getClothDefinition()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothDefinition > d(new edu::rpi::cats::utilities::clothsim::ClothDefinition());
	
	d->length = this->cloth_length;
	d->width = this->cloth_width;
	d->mass = this->cloth_mass;
	d->numX = this->numX;
	d->numY = this->numY;
	d->n_points = (uint32_t)this->numX * (uint32_t)this->numY;
	d->structure_stiffness = this->cloth_stiffness;
	d->bending_stiffness = this->cloth_bending_stiffness;
	

	return d;
}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothLinks > ClothSimImpl::getClothLinks()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothLinks > s(new edu::rpi::cats::utilities::clothsim::ClothLinks());

	btSoftBody::Node *node_address0 = &this->cloth->m_nodes.at(0);

	uint16_t *left_nodes = new uint16_t[this->cloth->m_links.size()];
	uint16_t *right_nodes = new uint16_t[this->cloth->m_links.size()];
	float *resting_lengths = new float[this->cloth->m_links.size()];
	float *stiffness = new float[this->cloth->m_links.size()];
	for (unsigned int i = 0; i < this->cloth->m_links.size(); ++i)
	{
		left_nodes[i] = this->cloth->m_links.at(i).m_n[0] - node_address0;
		right_nodes[i] = this->cloth->m_links.at(i).m_n[1] - node_address0;
		resting_lengths[i] = this->cloth->m_links.at(i).m_rl;
		stiffness[i] = this->cloth->m_links.at(i).m_bbending ? this->cloth_bending_stiffness : this->cloth_stiffness;
	}

	s->left_node = RobotRaconteur::AttachRRArray<uint16_t>(left_nodes, this->cloth->m_links.size(), true);
	s->right_node = RobotRaconteur::AttachRRArray<uint16_t>(right_nodes, this->cloth->m_links.size(), true);
	s->length = RobotRaconteur::AttachRRArray<float>(resting_lengths, this->cloth->m_links.size(), true);
	s->stiffness = RobotRaconteur::AttachRRArray<float>(stiffness, this->cloth->m_links.size(), true);

	return s;
}

int ClothSimImpl::initWorldAndCloth()
{
	std::cout << "Allocating dynamic world parameters (broadphase, dispatcher, solver)" << std::endl;
	// Using a Dynamic AABB Tree broadphase algorithm
	this->broadphase = new btDbvtBroadphase();
	// Using a Sweep and Prune broadphase algorithm
	//this->broadphase = new btAxisSweep3(btVector3(-100, -100, -100), btVector3(100, 100, 100));

	this->collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	this->dispatcher = new btCollisionDispatcher(collisionConfiguration);
	// Default constraint solver
	this->solver = new btSequentialImpulseConstraintSolver;

	std::cout << "Generating world" << std::endl;

	this->world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	this->world->setGravity(btVector3(0, 0, -9.81f));
	this->world_info = this->world->getWorldInfo();
	this->world_info.m_gravity = this->world->getGravity();
	this->world_info.m_maxDisplacement = 2.f * this->dt;
	this->world_info.m_sparsesdf.Initialize();
	std::cout << "Generating cloth" << std::endl;

	cloth = btSoftBodyHelpers::CreatePatch(world_info, btVector3(-this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, -this->cloth_length / 2.f, 0),
														btVector3(-this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
														btVector3(this->cloth_width / 2.f, this->cloth_length / 2.f, 0),
															this->numX, this->numY, 0, true);
	std::cout << "SoftBodyHelper generated cloth with..." << std::endl;
	std::cout << "Nodes: " << cloth->m_nodes.size() << std::endl;
	
	cloth->getCollisionShape()->setMargin(0.001f);

	btSoftBody::Material *structural_material = cloth->appendMaterial();
	structural_material->m_kLST = this->cloth_stiffness;
	structural_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	for (int i = 0; i < cloth->m_links.size(); i++)
		cloth->m_links.at(i).m_material = structural_material;

	btSoftBody::Material *bending_material = cloth->appendMaterial();
	bending_material->m_kLST = this->cloth_bending_stiffness;
	bending_material->m_flags -= btSoftBody::fMaterial::DebugDraw;
	cloth->generateBendingConstraints(2, bending_material);
	std::cout << "Links: " << cloth->m_links.size() << std::endl;
	

	cloth->updateConstants();

	double grasp_radius = 0.04f;

	for (int i = 0; i < this->cloth->m_nodes.size(); i++)
	{
		// testing within grasp00 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(0).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(0).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp00_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp10 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numX - 1).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numX - 1).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp10_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp01 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at((this->numY - 1)*this->numX).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at((this->numY - 1)*this->numX).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp01_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
		// testing within grasp11 origin
		if ((this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numY*this->numX - 1).m_x).length() <= grasp_radius)
		{
			node_relationship this_node;
			this_node.index = i;
			this_node.vec = (this->cloth->m_nodes.at(i).m_x - this->cloth->m_nodes.at(this->numY*this->numX - 1).m_x);
			this_node.x_des = this->cloth->m_nodes.at(i).m_x;
			grasp11_nodes.push_back(this_node);
			this->cloth->m_nodes.at(i).m_im = 0.f;
		}
	}

	


	cloth->setTotalMass(this->cloth_mass);
	cloth->m_cfg.kDP = this->cloth_damping;
	cloth->m_cfg.piterations = 1;// 10;// 5;
	cloth->updateConstants();

	world->addSoftBody(cloth);

	return cloth->m_nodes.size();
}

void ClothSimImpl::shutdown()
{
	if (broadphase == 0)
		return;

	th1.interrupt();
	th1.join();

	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	if (world == 0)
		return;

	world->removeSoftBody(cloth);
	delete cloth;


	delete world; 
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
	world = 0;
	solver = 0;
	dispatcher = 0;
	collisionConfiguration = 0;
	broadphase = 0;
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes00()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp00_nodes.size()];
	for (unsigned int i = 0; i < this->grasp00_nodes.size(); i++)
		g_nodes[i] = this->grasp00_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp00_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes10()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp10_nodes.size()];
	for (unsigned int i = 0; i < this->grasp10_nodes.size(); i++)
		g_nodes[i] = this->grasp10_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp10_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes01()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp01_nodes.size()];
	for (unsigned int i = 0; i < this->grasp01_nodes.size(); i++)
		g_nodes[i] = this->grasp01_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp01_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::get_grasped_nodes11()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > g;
	uint16_t *g_nodes = new uint16_t[this->grasp11_nodes.size()];
	for (unsigned int i = 0; i < this->grasp11_nodes.size(); i++)
		g_nodes[i] = this->grasp11_nodes.at(i).index;

	g = RobotRaconteur::AttachRRArray<uint16_t>(g_nodes, this->grasp11_nodes.size(), true);

	return g;
}
void ClothSimImpl::set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
}

void ClothSimImpl::start_recording(std::string record_name)
{
	if (this->record_stream.is_open())
		stop_recording();

	record_name += ".csv";
	this->record_stream.open(record_name.c_str());

	this->t = 0.f;
	this->_recording = true;
}

void ClothSimImpl::stop_recording()
{
	if (!this->record_stream.is_open())
		return;
	this->record_stream.close();

	this->_recording = false;
}

RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ClothSimImpl::getFaceStructure()
{
	boost::lock_guard<boost::mutex> guard(mtx_);


	RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > F;
	uint16_t *face_nodes = new uint16_t[3 * this->cloth->m_faces.size()];
	btSoftBody::Node *node_address0 = &this->cloth->m_nodes.at(0);

	for (int i = 0; i < this->cloth->m_faces.size(); ++i)
	{
		face_nodes[3 * i] = this->cloth->m_faces.at(i).m_n[0] - node_address0;
		face_nodes[3 * i + 1] = this->cloth->m_faces.at(i).m_n[1] - node_address0;
		face_nodes[3 * i + 2] = this->cloth->m_faces.at(i).m_n[2] - node_address0;
	}

	F = RobotRaconteur::AttachRRArray<uint16_t>(face_nodes, 3 * this->cloth->m_faces.size(), true);

	return F;
}

void ClothSimImpl::setClothStiffness(double stiffness, uint8_t piterations)
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	cloth->m_cfg.piterations = piterations;// 10;// 5;

	for (int i = 0, ni = cloth->m_links.size(); i < ni; ++i)
		cloth->m_links.at(i).m_material->m_kLST = stiffness;
	cloth->updateLinkConstants();

}

void ClothSimImpl::setNewGraspVelocities(double tstep, RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p, std::vector<node_relationship> &grasp_nodes)
{
	btMatrix3x3 Rot(p->R->ptr()[0], p->R->ptr()[1], p->R->ptr()[2],
		p->R->ptr()[3], p->R->ptr()[4], p->R->ptr()[5],
		p->R->ptr()[6], p->R->ptr()[7], p->R->ptr()[8]);
	btVector3 Tran(p->p->ptr()[0], p->p->ptr()[1], p->p->ptr()[2]);

	for (unsigned int i = 0; i < grasp_nodes.size(); ++i)
	{
		int idx = grasp_nodes.at(i).index;
		btVector3 xf = Tran + Rot * grasp_nodes.at(i).vec;
		grasp_nodes.at(i).v_des = (xf - this->cloth->m_nodes.at(idx).m_x) / tstep;
	}
}

void ClothSimImpl::stepForwardGraspPoints(double tstep, std::vector<node_relationship> grasp_nodes)
{
	for (unsigned int i = 0; i < grasp_nodes.size(); ++i)
	{
		int idx = grasp_nodes.at(i).index;
		this->cloth->m_nodes.at(idx).m_x += grasp_nodes.at(i).v_des * tstep;
	}
}

void ClothSimImpl::solveNodeCost()
{
	for (int i = 0; i < this->n_points; ++i)
		this->f[i] = 0.f;

	btSoftBody::Node *n0 = &this->cloth->m_nodes.at(0);
	for (int i = 0, ni = this->cloth->m_links.size(); i < ni; ++i)
	{
		btSoftBody::Link li = this->cloth->m_links.at(i);
		int a = li.m_n[0] - n0;
		int b = li.m_n[1] - n0;
		btScalar dx = (li.m_n[1]->m_x - li.m_n[0]->m_x).norm() - li.m_rl;
		float fi = dx*dx;
		this->f[a] += fi;
		this->f[b] += fi;
	}
}

void ClothSimImpl::solveSpringForces(btScalar structural_stiffness, btScalar bending_stiffness)
{
	// clear out current force magnitudes
	/*
	for (unsigned int i = 0; i < this->n_points; ++i)
	{
		this->fx[i] = this->fy[i] = 0.f;
		this->fz[i] = this->cloth->m_nodes.at(i).m_im ? 9.81f / this->cloth->m_nodes.at(i).m_im : 0.f;
	}

	for (int i = 0; i < this->cloth->m_links.size(); ++i)
	{
		btScalar k = this->cloth->m_links.at(i).m_bbending ? bending_stiffness : structural_stiffness;
		int a = this->cloth->m_links.at(i).m_n[0] - &this->cloth->m_nodes.at(0);
		int b = this->cloth->m_links.at(i).m_n[1] - &this->cloth->m_nodes.at(0);
		btVector3 dx = this->cloth->m_links.at(i).m_n[1]->m_x - this->cloth->m_links.at(i).m_n[0]->m_x;
		btVector3 f = k*(1.f - this->cloth->m_links.at(i).m_rl / dx.norm())*dx;
		this->fx[a] += f.x();
		this->fy[a] += f.y();
		this->fz[a] += f.z();
		this->fx[b] += -f.x();
		this->fy[b] += -f.y();
		this->fz[b] += -f.z();
	}*/
}

void ClothSimImpl::addSpringForces(btScalar structural_stiffness, btScalar bending_stiffness)
{
	for (unsigned int i = 0, ni = this->n_points; i < ni; ++i)
		this->spring_forces.at(i) = btVector3(0.f,0.f,0.f);
	

	for (int i = 0, ni = cloth->m_links.size(); i < ni; ++i)
	{
		btScalar k = this->cloth->m_links.at(i).m_bbending ? bending_stiffness : structural_stiffness;
		int a = this->cloth->m_links.at(i).m_n[0] - &this->cloth->m_nodes.at(0);
		int b = this->cloth->m_links.at(i).m_n[1] - &this->cloth->m_nodes.at(0);
		btVector3 dx = this->cloth->m_links.at(i).m_n[1]->m_x - this->cloth->m_links.at(i).m_n[0]->m_x;
		btVector3 f = k*(1.f - this->cloth->m_links.at(i).m_rl / dx.norm())*dx;
		
		this->spring_forces.at(a) += f;
		this->spring_forces.at(b) -= f;

		//this->fx[a] += f.x();
		//this->fy[a] += f.y();
		//this->fz[a] += f.z();
		//this->fx[b] += -f.x();
		//this->fy[b] += -f.y();
		//this->fz[b] += -f.z();
	}
	
	for (unsigned int i = 0, ni = this->n_points; i < ni; ++i)
		this->cloth->addForce(this->spring_forces.at(i), i);
}

void ClothSimImpl::setCameraPose(RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > pk)
{
	this->ROk = btMatrix3x3(pk->R->ptr()[0], pk->R->ptr()[1], pk->R->ptr()[2], 
						pk->R->ptr()[3], pk->R->ptr()[4], pk->R->ptr()[5], 
						pk->R->ptr()[6], pk->R->ptr()[7], pk->R->ptr()[8]);
	this->pOk_O = btVector3(pk->p->ptr()[0], pk->p->ptr()[1], pk->p->ptr()[2]);
}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::DepthImage > ClothSimImpl::getRenderedImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	renderDepthImage();

	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::DepthImage > Id(new edu::rpi::cats::utilities::clothsim::DepthImage());

	Id->width = this->image_width;
	Id->height = this->image_height;
	Id->data = RobotRaconteur::AttachRRArrayCopy<float>(this->image_data, Id->width * Id->height);

	return Id;
}

void ClothSimImpl::renderDepthImage()
{

	btVector3 V0, V1, V2, V01, V02;
	btVector3 u, v;
	btVector3 n;
	bool *data_set = new bool[this->image_width * this->image_height];
	memset(data_set, false, this->image_width * this->image_height * sizeof(bool));
	memset(this->image_data, 0, this->image_width * this->image_height * sizeof(float));

	
	if (pOk_O.norm() == 0 && ROk[0][0] == 1 && ROk[1][1] == 1 && ROk[2][2] == 1)
		return;

	for (int k = 0; k < this->cloth->m_faces.size(); ++k)
	{
		V0 = ROk.transpose()*(this->cloth->m_faces.at(k).m_n[0]->m_x - this->pOk_O);
		V1 = ROk.transpose()*(this->cloth->m_faces.at(k).m_n[1]->m_x - this->pOk_O);
		V2 = ROk.transpose()*(this->cloth->m_faces.at(k).m_n[2]->m_x - this->pOk_O);

		u[0] = this->K[0][0] * V0[0] / V0[2] + this->K[0][2];
		u[1] = this->K[0][0] * V1[0] / V1[2] + this->K[0][2];
		u[2] = this->K[0][0] * V2[0] / V2[2] + this->K[0][2];

		v[0] = this->K[1][1] * V0[1] / V0[2] + this->K[1][2];
		v[1] = this->K[1][1] * V1[1] / V1[2] + this->K[1][2];
		v[2] = this->K[1][1] * V2[1] / V2[2] + this->K[1][2];

		int u1 = std::max(0, (int)std::floor(u[u.minAxis()]));
		int v1 = std::max(0, (int)std::floor(v[v.minAxis()]));
		int u2 = std::min(this->image_width - 1, (int)std::ceil(u[u.maxAxis()]));
		int v2 = std::min(this->image_height - 1, (int)std::ceil(v[v.maxAxis()]));
		
		if (u2 <= u1 || v2 <= v1)
			continue;

		
		V01 = V1 - V0;
		V02 = V2 - V0;
		n = V01.cross(V02);
		btScalar v_scale = V01.dot(V02) * V01.dot(V02) - V01.dot(V01) * V02.dot(V02);
		btVector3 s_coeff = (V01.dot(V02) * V02 - V02.dot(V02) * V01) / v_scale;
		btVector3 t_coeff = (V01.dot(V02) * V01 - V01.dot(V01) * V02) / v_scale;

		for (int i = v1; i <= v2; ++i)
		{
			for (int j = u1; j <= u2; ++j)
			{
				btVector3 ell = btVector3(((float)j - K[0][2]) / K[0][0], ((float)i - K[1][2]) / K[1][1], 1.f);
				btScalar r = n.dot(V0) / n.dot(ell);
				btVector3 w = r * ell - V0;
				btScalar s = s_coeff.dot(w);
				btScalar t = t_coeff.dot(w);
				if (r < 0 || s < 0 || t < 0 || (s + t) > 1)
					continue;
				
				if (!data_set[i*this->image_width + j] || (r*ell.z() < this->image_data[i*this->image_width + j]))
				{
					this->image_data[i*this->image_width + j] = r*ell.z();
					data_set[i*this->image_width + j] = true;
				}
				
			}
		}

	}
	delete data_set;

}

void ClothSimImpl::write_data_to_file()
{
	this->record_stream << this->t;
	for (int i = 0; i < this->n_points; i++)
	{
		this->record_stream << ", " << this->cloth->m_nodes[i].m_x.x() << ", " << this->cloth->m_nodes[i].m_x.y() << ", " << this->cloth->m_nodes[i].m_x.z() << ", " << this->f[i];
		//this->record_stream << ", " << this->fx[i] << ", " << this->fy[i] << ", " << this->fz[i];
	}
	this->record_stream << std::endl;

}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState > ClothSimImpl::stepForwardSim(double tstep,
											RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p00,
											RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p10,
											RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p01,
											RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p11)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState> CState(new edu::rpi::cats::utilities::clothsim::ClothState());
	
	if (tstep > 0.f)
	{
		setNewGraspVelocities(tstep, p00, this->grasp00_nodes);
		setNewGraspVelocities(tstep, p10, this->grasp10_nodes);
		setNewGraspVelocities(tstep, p01, this->grasp01_nodes);
		setNewGraspVelocities(tstep, p11, this->grasp11_nodes);

		int steps = ceil(tstep / this->dt);
		btScalar time_left = tstep;
		for (int i = 0; i < steps; ++i)
		{
			btScalar sim_time = std::min(time_left, this->dt);

			stepForwardGraspPoints(sim_time, this->grasp00_nodes);
			stepForwardGraspPoints(sim_time, this->grasp10_nodes);
			stepForwardGraspPoints(sim_time, this->grasp01_nodes);
			stepForwardGraspPoints(sim_time, this->grasp11_nodes);

			//addSpringForces(.001f, .001f);
			this->world->stepSimulation(sim_time, 1, this->dt);
			//solveSpringForces(0.1f, 0.1f);
			time_left -= sim_time;
			this->t += sim_time;
			if (this->_recording)
				write_data_to_file();

		}
	}

	// Update local variables storing the mesh positions
	for (unsigned int i = 0; i < this->n_points; i++)
	{
		this->x[i] = this->cloth->m_nodes.at(i).m_x.x();
		this->y[i] = this->cloth->m_nodes.at(i).m_x.y();
		this->z[i] = this->cloth->m_nodes.at(i).m_x.z();
	}
	solveNodeCost();



	CState->numX = this->numX;
	CState->numY = this->numY;
	CState->n_points = this->n_points;
	CState->t = this->t;
	CState->x = RobotRaconteur::AttachRRArrayCopy<double>(this->x, this->n_points);
	CState->y = RobotRaconteur::AttachRRArrayCopy<double>(this->y, this->n_points);
	CState->z = RobotRaconteur::AttachRRArrayCopy<double>(this->z, this->n_points);
	CState->f = RobotRaconteur::AttachRRArrayCopy<double>(this->f, this->n_points);
	//CState->fx = RobotRaconteur::AttachRRArrayCopy<double>(this->fx, this->n_points);
	//CState->fy = RobotRaconteur::AttachRRArrayCopy<double>(this->fy, this->n_points);
	//CState->fz = RobotRaconteur::AttachRRArrayCopy<double>(this->fz, this->n_points);

	return CState;

}

RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState > ClothSimImpl::stepSimToConverge(
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p00,
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p10,
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p01,
	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::Pose > p11)
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothState> CState(new edu::rpi::cats::utilities::clothsim::ClothState());

	btScalar tstep = 0.25f;
	float cloth_cost = 100.f; // Arbitrarily large
	float cloth_cost_last;

	int k = 0;
	while (k < 15)
	{
		k++;
		setNewGraspVelocities(tstep, p00, this->grasp00_nodes);
		setNewGraspVelocities(tstep, p10, this->grasp10_nodes);
		setNewGraspVelocities(tstep, p01, this->grasp01_nodes);
		setNewGraspVelocities(tstep, p11, this->grasp11_nodes);

		int steps = ceil(tstep / this->dt);
		btScalar time_left = tstep;
		for (int i = 0; i < steps; ++i)
		{
			btScalar sim_time = std::min(time_left, this->dt);

			stepForwardGraspPoints(sim_time, this->grasp00_nodes);
			stepForwardGraspPoints(sim_time, this->grasp10_nodes);
			stepForwardGraspPoints(sim_time, this->grasp01_nodes);
			stepForwardGraspPoints(sim_time, this->grasp11_nodes);

			this->world->stepSimulation(sim_time, 1, this->dt);
			time_left -= sim_time;
			this->t += sim_time;
		}

		solveNodeCost();
		cloth_cost_last = cloth_cost;
		cloth_cost = 0.f;
		for (int i = 0; i < this->n_points; ++i)
			cloth_cost += this->f[i];
		if (fabs(cloth_cost - cloth_cost_last) < .001)
			break;
	}
	std::cout << "Converged in " << k << " steps, change: " << fabs(cloth_cost - cloth_cost_last) << std::endl;

	// Update local variables storing the mesh positions
	for (unsigned int i = 0; i < this->n_points; i++)
	{
		this->x[i] = this->cloth->m_nodes.at(i).m_x.x();
		this->y[i] = this->cloth->m_nodes.at(i).m_x.y();
		this->z[i] = this->cloth->m_nodes.at(i).m_x.z();
	}



	CState->numX = this->numX;
	CState->numY = this->numY;
	CState->n_points = this->n_points;
	CState->t = this->t;
	CState->x = RobotRaconteur::AttachRRArrayCopy<double>(this->x, this->n_points);
	CState->y = RobotRaconteur::AttachRRArrayCopy<double>(this->y, this->n_points);
	CState->z = RobotRaconteur::AttachRRArrayCopy<double>(this->z, this->n_points);
	CState->f = RobotRaconteur::AttachRRArrayCopy<double>(this->f, this->n_points);

	return CState;
}