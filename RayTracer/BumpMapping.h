// Simple bump mapping helper that perturbs hInfo.N using a bump texture.
#pragma once

#include "materials.h"
#include "texture.h"
#include <algorithm>
#include <cmath>

inline void BuildBasisFromNormal(const Vec3f& n, Vec3f& t, Vec3f& b) {
	if (std::fabs(n.x) > 0.5f) t = Vec3f(0.0f, 1.0f, 0.0f) ^ n;
	else                       t = Vec3f(1.0f, 0.0f, 0.0f) ^ n;
	t.Normalize();
	b = n ^ t;
	b.Normalize();
}

inline void ApplyBumpMapping(const Material* mtl, HitInfo& h) {
	if (!mtl) return;
	const TextureMap* bump = mtl->GetBumpMap(h.mtlID);
	if (!bump) return;

	Vec3f N = h.N;
	if (N.LengthSquared() <= 0.0f) return;
	N.Normalize();

	Vec3f T = h.dpdu;
	Vec3f B = h.dpdv;
	if (T.LengthSquared() < 1e-10f || B.LengthSquared() < 1e-10f) {
		BuildBasisFromNormal(N, T, B);
	} else {
		T -= N * (T % N);
		if (T.LengthSquared() < 1e-10f) BuildBasisFromNormal(N, T, B);
		else {
			T.Normalize();
			B -= N * (B % N);
			B -= T * (B % T);
			if (B.LengthSquared() < 1e-10f) B = N ^ T;
			B.Normalize();
		}
	}

	float du = 0.001f, dv = 0.001f;
	if (const TextureFile* tf = dynamic_cast<const TextureFile*>(bump->GetTexture())) {
		int w = tf->Width();
		int hgt = tf->Height();
		int maxDim = std::max(w, hgt);
		if (maxDim > 0) {
			du = dv = 1.0f / float(maxDim);
		}
	}

	const float h0 = bump->Eval(h.uvw).Luma1();
	const float hx = bump->Eval(h.uvw + Vec3f(du, 0.0f, 0.0f)).Luma1();
	const float hy = bump->Eval(h.uvw + Vec3f(0.0f, dv, 0.0f)).Luma1();

	const float invDu = (std::fabs(du) > 1e-6f) ? 1.0f / du : 0.0f;
	const float invDv = (std::fabs(dv) > 1e-6f) ? 1.0f / dv : 0.0f;
	
	// Get bump intensity from material (defaults to 1.0 if not set)
	const float bumpScale = mtl->GetBumpIntensity(h.mtlID);
	
	const float dHdU = (hx - h0) * invDu * bumpScale;
	const float dHdV = (hy - h0) * invDv * bumpScale;

	Vec3f nTangent(-dHdU, -dHdV, 1.0f);
	if (nTangent.LengthSquared() > 0.0f) nTangent.Normalize();

	Vec3f bumped = T * nTangent.x + B * nTangent.y + N * nTangent.z;
	if (bumped.LengthSquared() > 0.0f) h.N = bumped.GetNormalized();
}
