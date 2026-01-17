#include <algorithm>
#include <cmath>
#include <limits>

#include "ShadingConfig.h"

#if !SHADING_USE_LEGACY

#include "materials.h"
#include "lights.h"
#include "renderer.h"
#include "scene.h"
#include "rng.h"

using std::max;
using std::min;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float Clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
static inline float Luminance(Color const& c) { return c.Luma1(); }

static inline Vec3f SafeNormalize(Vec3f v) {
	const float len2 = v.LengthSquared();
	if (len2 <= 0.0f) return Vec3f(0, 0, 0);
	return v / std::sqrt(len2);
}

static inline void BuildONB(Vec3f const& n, Vec3f& u, Vec3f& v) {
	if (std::fabs(n.x) > 0.5f) u = Vec3f(0, 1, 0) ^ n;
	else                         u = Vec3f(1, 0, 0) ^ n;
	u.Normalize();
	v = n ^ u;
	v.Normalize();
}

static inline Vec3f Reflect(Vec3f const& w, Vec3f const& n) {
	// Mirror reflection for an incident vector w (pointing toward the surface)
	// as in GLSL: r = w - 2 * dot(w, n) * n
	return w - 2.0f * (w % n) * n;
}

static inline float TransmissionScale(float eta, float cosI, float cosT) {
	float scale = eta * eta;
	return std::min(scale, 10.0f);
}

static inline float FresnelDielectric(float cosThetaI, float etaI, float etaT) {
	cosThetaI = Clamp01(std::fabs(cosThetaI));
	float eta = etaI / etaT;
	float sin2T = eta * eta * (1.0f - cosThetaI * cosThetaI);
	// Total internal reflection
	if (sin2T >= 1.0f) return 1.0f;

	float cosThetaT = std::sqrt(max(0.0f, 1.0f - sin2T));
	float rParl = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
	float rPerp = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
	return Clamp01(0.5f * (rParl * rParl + rPerp * rPerp));
}


// Point light sampling / intersection
bool PointLight::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const {
	if (size <= 0.0f) return false;

	Vec3f oc = ray.p - position;
	float b = oc % ray.dir;
	float c = oc.LengthSquared() - size * size;
	float disc = b * b - c;
	if (disc < 0.0f) return false;

	float t = -b - std::sqrt(std::max(0.0f, disc));
	if (t < 0.0f) t = -b + std::sqrt(std::max(0.0f, disc));
	if (t < 0.0f) return false;

	hInfo.z = t;
	hInfo.p = ray.p + ray.dir * t;
	hInfo.N = SafeNormalize(hInfo.p - position);
	hInfo.GN = hInfo.N;
	hInfo.mtlID = 0;
	hInfo.light = true;
	hInfo.front = ((ray.dir % hInfo.N) <= 0.0f);
	return true;
}

void PointLight::RandomPhoton(RNG& rng, Ray& r, Color& c) const {
	// Uniformly sample a point on the emitter (sphere or point) and a uniform direction.
	Vec3f pos = position;
	if (size > 0.0f) {
		float u1 = rng.RandomFloat();
		float u2 = rng.RandomFloat();
		float z = 1.0f - 2.0f * u1;
		float rxy = std::sqrt(max(0.0f, 1.0f - z * z));
		float phi = 2.0f * float(M_PI) * u2;
		pos = position + size * Vec3f(rxy * std::cos(phi), rxy * std::sin(phi), z);
	}

	float u1 = rng.RandomFloat();
	float u2 = rng.RandomFloat();
	float z = 1.0f - 2.0f * u1;
	float rxy = std::sqrt(max(0.0f, 1.0f - z * z));
	float phi = 2.0f * float(M_PI) * u2;
	Vec3f dir(rxy * std::cos(phi), rxy * std::sin(phi), z);

	// If sampling from a spherical emitter, make photons leave the surface.
	if (size > 0.0f) {
		Vec3f n = SafeNormalize(pos - position);
		if (dir % n < 0.0f) dir = -dir;
	}

	r.p = pos;
	r.dir = dir;
	c = intensity * 4.0f * float(M_PI);
}

bool PointLight::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const {
	si.SetVoid();
	Vec3f P = sInfo.P();
	Vec3f toC = position - P;
	float dc = toC.Length();
	if (dc < 1e-6f) return false;

	if (size <= 0.0f) {
		dir = toC / dc;
		float att_factor = (attenuation == 0.0f) ? 1.0f : attenuation * attenuation;
		si.mult = intensity / (att_factor * dc * dc);
		si.prob = 1.0f;
		si.dist = dc;
		si.lobe = DirSampler::Lobe::ALL;
		return true;
	}

	// Sample a point uniformly on the sphere
	float u1 = sInfo.RandomFloat();
	float u2 = sInfo.RandomFloat();
	float z = 1.0f - 2.0f * u1;
	float rxy = std::sqrt(max(0.0f, 1.0f - z * z));
	float phi = 2.0f * float(M_PI) * u2;
	Vec3f onS = Vec3f(rxy * std::cos(phi), rxy * std::sin(phi), z);
	Vec3f Lpos = position + size * onS;

	Vec3f wi = Lpos - P;
	float dist = wi.Length();
	if (dist < 1e-6f) return false;
	dir = wi / dist;

	Vec3f nL = onS; // already normalized
	float cosLight = max(0.0f, -nL % dir);
	if (cosLight <= 0.0f) return false;

	float area = 4.0f * float(M_PI) * size * size;
	// We reject back-facing samples, so the effective pdf doubles on the visible hemisphere.
	float pdfArea = 2.0f / area;

	// Light radiance: intensity / (pi * r^2)
	Color radiance = intensity / (float(M_PI) * size * size);
	float att_factor = (attenuation == 0.0f) ? 1.0f : attenuation * attenuation;
	si.mult = radiance / att_factor;
	si.prob = pdfArea;
	si.dist = dist;
	si.lobe = DirSampler::Lobe::ALL;
	return si.prob > 0.0f;
}

void PointLight::GetSampleInfo(SamplerInfo const& sInfo, Vec3f const& dir, DirSampler::Info& si) const {
	si.SetVoid();
	Vec3f P = sInfo.P();
	Vec3f d = SafeNormalize(dir);

	if (size <= 0.0f) {
		Vec3f toL = position - P;
		float dist = toL.Length();
		if (dist < 1e-6f) return;
		if (SafeNormalize(toL) % d < 1.0f - 1e-4f) return; // not pointing to the light
		float att_factor = (attenuation == 0.0f) ? 1.0f : attenuation * attenuation;
		si.mult = intensity / (att_factor * dist * dist);
		si.prob = 1.0f;
		si.dist = dist;
		si.lobe = DirSampler::Lobe::ALL;
		return;
	}

	// Ray-sphere intersection for renderable light
	Vec3f oc = P - position;
	float b = oc % d;
	float c = oc.LengthSquared() - size * size;
	float disc = b * b - c;
	if (disc < 0.0f) return;
	float t = -b - std::sqrt(std::max(0.0f, disc));
	if (t <= 1e-6f) return;

	Vec3f hit = P + d * t;
	Vec3f nL = SafeNormalize(hit - position);
	float cosLight = max(0.0f, -nL % d);
	if (cosLight <= 0.0f) return;

	float area = 4.0f * float(M_PI) * size * size;
	float pdfArea = 2.0f / area; // only the visible hemisphere is considered

	// Light radiance: intensity / (4 * pi * r^2)
	Color radiance = intensity / (float(M_PI) * size * size);
	float att_factor = (attenuation == 0.0f) ? 1.0f : attenuation * attenuation;
	si.mult = radiance / att_factor;
	si.prob = pdfArea;
	si.dist = t;
	si.lobe = DirSampler::Lobe::ALL;
}

// Phong / Blinn materials (shared helpers)
struct PhongContext {
	Vec3f N;
	Vec3f V;
	Vec3f R;
	Color kd;
	Color ks;
	Color kr;
	Color kt;
	float alpha;
	float ior;
	float wd, ws, wt, wr, total;
};

enum class PhongType { Phong, Blinn };

static PhongContext BuildPhongContext(SamplerInfo const& sInfo, MtlBasePhongBlinn const& mtl) {
	PhongContext ctx;
	ctx.N = sInfo.N();
	ctx.N.Normalize();
	if (!sInfo.IsFront()) ctx.N = -ctx.N;
	ctx.V = SafeNormalize(sInfo.V());
	ctx.R = SafeNormalize(Reflect(-ctx.V, ctx.N));
	ctx.kd = sInfo.Eval(mtl.Diffuse());
	ctx.ks = sInfo.Eval(mtl.Specular());
	ctx.kr = sInfo.Eval(mtl.Reflection());
	ctx.kt = sInfo.Eval(mtl.Refraction());
	float gloss = max(0.001f, sInfo.Eval(mtl.Glossiness()));
	ctx.alpha = gloss;
	ctx.ior = mtl.IOR(sInfo.MaterialID());

	ctx.kd.ClampMax(1.0f);
	ctx.ks.ClampMax(1.0f);
	ctx.kr.ClampMax(1.0f);
	ctx.kt.ClampMax(1.0f);

	// For dielectrics (no explicit mirror/refraction), keep specular highlights in a plausible F0 range
	// so white specular doesn't overpower the base color and wash out materials like the cereal.
	float krMax = ctx.kr.Max();
	float ktMax = ctx.kt.Max();
	if (krMax <= 0.0f && ktMax <= 0.0f) {
		Color maxKs(0.08f); // ~F0 of common dielectrics
		if (ctx.ks.r > maxKs.r || ctx.ks.g > maxKs.g || ctx.ks.b > maxKs.b) {
			ctx.ks.r = std::min(ctx.ks.r, maxKs.r);
			ctx.ks.g = std::min(ctx.ks.g, maxKs.g);
			ctx.ks.b = std::min(ctx.ks.b, maxKs.b);
		}
	}

	// Conserve energy while preserving explicit mirror/refraction terms:
	// reserve energy for kr/kt first, then fit kd+ks into the leftover.
	float reserve = std::min(1.0f, krMax + ktMax);
	float available = std::max(0.0f, 1.0f - reserve);
	Color kdks = ctx.kd + ctx.ks;
	float kdksMax = std::max({ kdks.r, kdks.g, kdks.b });
	if (kdksMax > 0.0f && kdksMax > available) {
		float scale = available / kdksMax;
		ctx.kd *= scale;
		ctx.ks *= scale;
	}

	ctx.wd = Luminance(ctx.kd);
	ctx.ws = Luminance(ctx.ks);
	ctx.wt = Luminance(ctx.kt);
	ctx.wr = Luminance(ctx.kr);
	ctx.total = ctx.wd + ctx.ws + ctx.wt + ctx.wr;
	return ctx;
}

static bool SamplePhongBlinn(PhongContext const& ctx, PhongType type, SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) {
	si.SetVoid();
	float etaI = sInfo.IsFront() ? 1.0f : ctx.ior;
	float etaT = sInfo.IsFront() ? ctx.ior : 1.0f;
	float F = (ctx.ior > 0.0f && ctx.kt.Max() > 0.0f) ? FresnelDielectric(std::fabs(ctx.N % ctx.V), etaI, etaT) : 0.0f;
	Color fresnelRef = ctx.kt * F;
	Color transColor = ctx.kt * (1.0f - F);

	// Keep glossy specular separate; route Fresnel to reflection lobe so we don't miss rim highlights
	Color specColor = ctx.ks;
	if (ctx.kr.Max() > 0.0f) {
		// Mirror reflection is active, so disable glossy specular to avoid double-counting
		specColor = Color(0.0f);
	}
	specColor.ClampMax(1.0f);

	float pd = Luminance(ctx.kd);
	float ps = Luminance(specColor);
	float pr = Luminance(ctx.kr + fresnelRef); // always include Fresnel reflection so grazing angles aren't dark
	float pt = Luminance(transColor);
	float total = pd + ps + pr + pt;
	if (total <= 0.0f) return false;

	float r = sInfo.RandomFloat() * total;

	if (r < pd) {
		// Diffuse: cosine-weighted
		Vec3f U, Vb;
		BuildONB(ctx.N, U, Vb);
		float u1 = sInfo.RandomFloat();
		float u2 = sInfo.RandomFloat();
		float rxy = std::sqrt(u1);
		float phi = 2.0f * float(M_PI) * u2;
		float x = rxy * std::cos(phi);
		float y = rxy * std::sin(phi);
		float z = std::sqrt(max(0.0f, 1.0f - u1));
		dir = SafeNormalize(U * x + Vb * y + ctx.N * z);

		float cosN = max(0.0f, ctx.N % dir);
		float pdf = cosN / float(M_PI);
		si.mult = ctx.kd * (1.0f / float(M_PI)) * cosN;
		si.prob = (pd / total) * pdf;
		si.dist = 0.0f;
		si.lobe = DirSampler::Lobe::DIFFUSE;
		return si.prob > 0.0f && si.mult.Max() > 0.0f;
	}
	else if (r < pd + ps) {
		if (type == PhongType::Phong) {
			// Glossy reflection using Phong exponent about the reflection direction
			Vec3f U, Vb;
			BuildONB(ctx.R, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosT = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinT = std::sqrt(max(0.0f, 1.0f - cosT * cosT));
			dir = SafeNormalize(U * (std::cos(phi) * sinT) + Vb * (std::sin(phi) * sinT) + ctx.R * cosT);

			float cosR = max(0.0f, ctx.R % dir);
			float cosN = max(0.0f, ctx.N % dir);
			float pdfDir = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);

			si.mult = specColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha) * cosN;
			si.prob = (ps / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::SPECULAR;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
		else {
			// Blinn: sample the half-vector around the normal and reflect about it
			Vec3f U, Vb;
			BuildONB(ctx.N, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosT = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinT = std::sqrt(max(0.0f, 1.0f - cosT * cosT));
			Vec3f h = SafeNormalize(U * (std::cos(phi) * sinT) + Vb * (std::sin(phi) * sinT) + ctx.N * cosT);
			dir = SafeNormalize(Reflect(-ctx.V, h));

			float NoL = max(0.0f, ctx.N % dir);
			float NoH = max(0.0f, ctx.N % h);
			float VoH = max(0.0f, ctx.V % h);
			if (NoL <= 0.0f || NoH <= 0.0f || VoH <= 1e-6f) return false;

			float pdfH = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
			float pdfDir = pdfH / (4.0f * max(1e-6f, VoH));

			// Divide by VoH to cancel the Jacobian term that appears in the half-vector PDF
			float invVoH = 1.0f / max(1e-6f, VoH);
			Color fSpec = specColor * ((ctx.alpha + 8.0f) / (8.0f * float(M_PI))) * std::pow(NoH, ctx.alpha) * invVoH;

			si.mult = fSpec;
			si.prob = (ps / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::SPECULAR;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
	}
	else if (r < pd + ps + pr) {
		// Glossy reflection lobe sampled around the mirror reflection direction
		Color reflColor = ctx.kr + fresnelRef;
		reflColor.ClampMax(1.0f);

		if (type == PhongType::Phong) {
			Vec3f U, Vb;
			BuildONB(ctx.R, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosT = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinT = std::sqrt(max(0.0f, 1.0f - cosT * cosT));
			dir = SafeNormalize(U * (std::cos(phi) * sinT) + Vb * (std::sin(phi) * sinT) + ctx.R * cosT);

			float cosR = max(0.0f, ctx.R % dir);
			float cosN = max(0.0f, ctx.N % dir);
			float pdfDir = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);

			float brdf = ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);
			si.mult = reflColor * brdf;
			si.prob = (pr / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::SPECULAR;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
		else {
			// Blinn: sample the half-vector around the normal and reflect about it
			Vec3f U, Vb;
			BuildONB(ctx.N, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosT = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinT = std::sqrt(max(0.0f, 1.0f - cosT * cosT));
			Vec3f h = SafeNormalize(U * (std::cos(phi) * sinT) + Vb * (std::sin(phi) * sinT) + ctx.N * cosT);
			
			// Reflect V about H: dir = 2*(V·H)*H - V
			float VoH = ctx.V % h;
			dir = SafeNormalize(2.0f * VoH * h - ctx.V);

			float NoL = max(0.0f, ctx.N % dir);
			float NoH = max(0.0f, ctx.N % h);
			if (NoL <= 0.0f || NoH <= 0.0f || VoH <= 1e-6f) return false;

			float pdfH = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
			float pdfDir = pdfH / (4.0f * max(1e-6f, VoH));

			// Remove the extra VoH geometry term introduced by the half-vector PDF
			float invVoH = 1.0f / max(1e-6f, VoH);
			Color fSpec = reflColor * ((ctx.alpha + 8.0f) / (8.0f * float(M_PI))) * std::pow(NoH, ctx.alpha) * invVoH;

			si.mult = fSpec;
			si.prob = (pr / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::SPECULAR;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
	}
	else {
		float eta = sInfo.IsFront() ? 1.0f / ctx.ior : ctx.ior;
		Vec3f I = -ctx.V;
		if (type == PhongType::Phong) {
			// Transmission with glossy lobe around the ideal refraction direction (Phong)
			float cosI = -(ctx.N % I); // signed
			float k = 1.0f - eta * eta * (1.0f - cosI * cosI);
			if (k < 0.0f) {
				// Total internal reflection: reflect all transmission energy
				dir = SafeNormalize(Reflect(-ctx.V, ctx.N));
				// Redirect only the transmission energy; keep the weight aligned with the transmission lobe PDF
				si.mult = transColor;
				si.prob = (pt / total);
				si.dist = -1.0f; // mark as delta event
				si.lobe = DirSampler::Lobe::SPECULAR;
				return si.prob > 0.0f && si.mult.Max() > 0.0f;
			}
			float cosTIdeal = std::sqrt(k);
			Vec3f T = SafeNormalize(eta * I + (eta * cosI - cosTIdeal) * ctx.N);

			// Sample a Phong-like lobe around the ideal refraction direction
			Vec3f U, Vb; BuildONB(T, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosTheta = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinTheta = std::sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
			dir = SafeNormalize(U * (std::cos(phi) * sinTheta) + Vb * (std::sin(phi) * sinTheta) + T * cosTheta);

			float cosT = max(0.0f, T % dir);
			float absCosN = std::fabs(ctx.N % dir);
			if (cosT <= 0.0f || absCosN <= 0.0f) return false;

			float pdfDir = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosT, ctx.alpha);
			float etaScale = TransmissionScale(eta, cosI, absCosN);
			Color fTrans = transColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosT, ctx.alpha);

			si.mult = fTrans * etaScale;
			si.prob = (pt / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::TRANSMISSION;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
		else {
			// Transmission using a Blinn-style microfacet half-vector
			Vec3f U, Vb; BuildONB(ctx.N, U, Vb);
			float u1 = sInfo.RandomFloat();
			float u2 = sInfo.RandomFloat();
			float phi = 2.0f * float(M_PI) * u1;
			float cosTheta = std::pow(u2, 1.0f / (ctx.alpha + 1.0f));
			float sinTheta = std::sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
			Vec3f h = SafeNormalize(U * (std::cos(phi) * sinTheta) + Vb * (std::sin(phi) * sinTheta) + ctx.N * cosTheta);
			if ((ctx.V % h) < 0.0f) h = -h;

			float cosI = -(h % I);
			float k = 1.0f - eta * eta * (1.0f - cosI * cosI);
			if (k < 0.0f) {
				dir = SafeNormalize(Reflect(-ctx.V, h));
				si.mult = transColor;
				si.prob = (pt / total);
				si.dist = -1.0f; // mark as delta event
				si.lobe = DirSampler::Lobe::SPECULAR;
				return si.prob > 0.0f && si.mult.Max() > 0.0f;
			}

			float cosT = std::sqrt(k);
			dir = SafeNormalize(eta * I + (eta * cosI - cosT) * h);

			float absCosN = std::fabs(ctx.N % dir);
			float VoH = ctx.V % h;
			float DoH = dir % h;
			if (absCosN <= 0.0f || VoH <= 1e-6f || VoH * DoH >= 0.0f) return false;

			float denom = eta * VoH + DoH;
			if (std::fabs(denom) <= 1e-6f) return false;

			float pdfH = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(max(0.0f, ctx.N % h), ctx.alpha);
			float dwh_dwi = std::abs((eta * eta * DoH) / (denom * denom));
			float pdfDir = pdfH * dwh_dwi;
			if (pdfDir <= 0.0f) return false;

			float etaScale = TransmissionScale(eta, cosI, absCosN);
			float NoH = max(0.0f, ctx.N % h);
			Color fTrans = transColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
			float factor = std::fabs(DoH * VoH) / max(1e-6f, std::fabs(ctx.N % ctx.V) * denom * denom);

			si.mult = fTrans * etaScale * factor;
			si.prob = (pt / total) * pdfDir;
			si.dist = 0.0f;
			si.lobe = DirSampler::Lobe::TRANSMISSION;
			return si.prob > 0.0f && si.mult.Max() > 0.0f;
		}
	}
}

static void EvaluatePhongBlinn(PhongContext const& ctx, PhongType type, SamplerInfo const& sInfo, Vec3f const& dirIn, DirSampler::Info& si) {
	si.SetVoid();
	float etaI = sInfo.IsFront() ? 1.0f : ctx.ior;
	float etaT = sInfo.IsFront() ? ctx.ior : 1.0f;
	float F = (ctx.ior > 0.0f && ctx.kt.Max() > 0.0f) ? FresnelDielectric(std::fabs(ctx.N % ctx.V), etaI, etaT) : 0.0f;
	Color fresnelRef = ctx.kt * F;
	Color transColor = ctx.kt * (1.0f - F);

	// Keep glossy specular separate; route Fresnel to reflection lobe so rim energy isn't lost
	Color specColor = ctx.ks;
	if (ctx.kr.Max() > 0.0f) {
		specColor = Color(0.0f);
	}
	specColor.ClampMax(1.0f);

	Vec3f dir = SafeNormalize(dirIn);
	float cosN = ctx.N % dir;

	Color mult(0.0f);
	float pdf = 0.0f;

	if (cosN > 0.0f) {
		// Compute lobe weights
		float pr = Luminance(ctx.kr + fresnelRef); // always include Fresnel reflection
		float ps = Luminance(specColor);
		float pd = Luminance(ctx.kd);
		float pt = Luminance(transColor);
		float total = pd + ps + pr + pt;
		if (total <= 0.0f) { si.SetVoid(); return; }

		float pdfDiff = pd > 0.0f ? (cosN / float(M_PI)) * (pd / total) : 0.0f;

		// Evaluate glossy reflection lobe (same as specular but using ctx.kr instead of ctx.ks)
		float pdfRefl = 0.0f;
		Color fRefl(0.0f);
		if (pr > 0.0f) {
			Color reflColor = ctx.kr + fresnelRef;
			reflColor.ClampMax(1.0f);

			if (type == PhongType::Phong) {
				float cosR = max(0.0f, ctx.R % dir);
				if (cosR > 0.0f) {
					pdfRefl = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);
					pdfRefl *= pr / total;
					fRefl = reflColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);
				}
			}
			else {
				Vec3f H = SafeNormalize(ctx.V + dir);
				float NoH = max(0.0f, ctx.N % H);
				float VoH = max(0.0f, ctx.V % H);
				if (NoH > 0.0f && VoH > 1e-6f) {
					float pdfH = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
					float pdfDir = pdfH / (4.0f * max(1e-6f, VoH));
					pdfRefl = pdfDir * (pr / total);
					float invVoH = 1.0f / max(1e-6f, VoH);
					fRefl = reflColor * ((ctx.alpha + 8.0f) / (8.0f * float(M_PI))) * std::pow(NoH, ctx.alpha) * invVoH;
				}
			}
		}

		float pdfSpec = 0.0f;
		bool  hasSpec = false;
		float blinnNoH = 0.0f;
		float blinnVoH = 0.0f;
		if (type == PhongType::Blinn) {
			Vec3f H = SafeNormalize(ctx.V + dir);
			blinnNoH = max(0.0f, ctx.N % H);
			blinnVoH = max(0.0f, ctx.V % H);
		}

		if (type == PhongType::Phong) {
			float cosR = max(0.0f, ctx.R % dir);
			if (ps > 0.0f && cosR > 0.0f) {
				pdfSpec = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);
				pdfSpec *= ps / total;
				hasSpec = true;
			}
		}
		else {
			if (ps > 0.0f && blinnNoH > 0.0f && blinnVoH > 1e-6f) {
				float pdfDir = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(blinnNoH, ctx.alpha);
				pdfDir /= (4.0f * max(1e-6f, blinnVoH));
				pdfSpec = pdfDir * (ps / total);
				hasSpec = true;
			}
		}

		Color fDiff = (pd > 0.0f) ? ctx.kd * (1.0f / float(M_PI)) : Color(0.0f);
		Color fSpec(0.0f);
		if (type == PhongType::Phong) {
			float cosR = max(0.0f, ctx.R % dir);
			if (ps > 0.0f && cosR > 0.0f) {
				fSpec = specColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosR, ctx.alpha);
			}
		}
		else {
			if (ps > 0.0f && blinnNoH > 0.0f && blinnVoH > 1e-6f) {
				float invVoH = 1.0f / max(1e-6f, blinnVoH);
				fSpec = specColor * ((ctx.alpha + 8.0f) / (8.0f * float(M_PI))) * std::pow(blinnNoH, ctx.alpha) * invVoH;
			}
		}

		// Apply the cosine geometry term once (diffuse + glossy); specular uses 1/VoH to cancel the half-vector Jacobian
		if (type == PhongType::Blinn) mult = (fRefl + fDiff + fSpec) * cosN;
		else                          mult = fRefl + (fDiff + fSpec) * cosN;

		pdf = pdfDiff + pdfSpec + pdfRefl;

		bool hasRefl = (pdfRefl > 0.0f);
		if ((hasSpec || hasRefl) && pd > 0.0f) si.lobe = DirSampler::Lobe::ALL;
		else if (hasSpec || hasRefl)           si.lobe = DirSampler::Lobe::SPECULAR;
		else                                   si.lobe = DirSampler::Lobe::DIFFUSE;
	}
	else {
		float pd = Luminance(ctx.kd);
		float ps = Luminance(specColor);
		float pr = (ctx.kr.Max() > 0.0f) ? Luminance(ctx.kr + fresnelRef) : 0.0f;
		float pt = Luminance(transColor);
		float total = pd + ps + pr + pt;
		if (total <= 0.0f) { si.SetVoid(); return; }

		if (pt > 0.0f) {
			float eta = sInfo.IsFront() ? 1.0f / ctx.ior : ctx.ior;
			float absCosWo = std::fabs(ctx.N % ctx.V);
			float absCosWi = std::fabs(ctx.N % dir);
			if (absCosWi <= 0.0f || absCosWo <= 0.0f) { si.SetVoid(); return; }

			if (type == PhongType::Phong) {
				Vec3f I = -ctx.V;
				float cosI = -(ctx.N % I);
				float k = 1.0f - eta * eta * (1.0f - cosI * cosI);
				if (k < 0.0f) { si.SetVoid(); return; }
				float cosTIdeal = std::sqrt(k);
				Vec3f T = SafeNormalize(eta * I + (eta * cosI - cosTIdeal) * ctx.N);

				float cosT = max(0.0f, T % dir);
				if (cosT <= 0.0f) { si.SetVoid(); return; }

				float pdfDir = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(cosT, ctx.alpha);
				float etaScale = TransmissionScale(eta, cosI, absCosWi);
				Color fTrans = transColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(cosT, ctx.alpha);

				mult = fTrans * etaScale;
				pdf = (pt / total) * pdfDir;
				si.lobe = DirSampler::Lobe::TRANSMISSION;
			}
			else {
				Vec3f H = SafeNormalize(ctx.V + dir * eta);
				if ((ctx.N % H) < 0.0f) H = -H;

				float NoH = max(0.0f, ctx.N % H);
				if (NoH <= 0.0f) { si.SetVoid(); return; }

				float VoH = ctx.V % H;
				float WiH = dir % H;
				if (VoH * WiH >= 0.0f) { si.SetVoid(); return; }

				float denom = eta * VoH + WiH;
				if (std::fabs(denom) <= 1e-6f) { si.SetVoid(); return; }

				float pdfH = ((ctx.alpha + 1.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
				float dwh_dwi = std::abs((eta * eta * WiH) / (denom * denom));
				float pdfDir = pdfH * dwh_dwi;

				float etaScale = TransmissionScale(eta, absCosWo, absCosWi);
				Color fTrans = transColor * ((ctx.alpha + 2.0f) / (2.0f * float(M_PI))) * std::pow(NoH, ctx.alpha);
				float factor = std::fabs(WiH * VoH) / max(1e-6f, absCosWo * denom * denom);

				mult = fTrans * etaScale * factor;
				pdf = (pt / total) * pdfDir;
				si.lobe = DirSampler::Lobe::TRANSMISSION;
			}
		}
	}

	if (pdf <= 0.0f || mult.Max() <= 0.0f) { si.SetVoid(); return; }
	si.mult = mult;
	si.prob = pdf;
	si.dist = 0.0f;
}

bool MtlBlinn::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const
{
	auto ctx = BuildPhongContext(sInfo, *this);
	return SamplePhongBlinn(ctx, PhongType::Blinn, sInfo, dir, si);
}

void MtlBlinn::GetSampleInfo(SamplerInfo const& sInfo, Vec3f const& dir, DirSampler::Info& si) const
{
	auto ctx = BuildPhongContext(sInfo, *this);
	EvaluatePhongBlinn(ctx, PhongType::Blinn, sInfo, dir, si);
}

bool MtlPhong::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const
{
	auto ctx = BuildPhongContext(sInfo, *this);
	return SamplePhongBlinn(ctx, PhongType::Phong, sInfo, dir, si);
}

void MtlPhong::GetSampleInfo(SamplerInfo const& sInfo, Vec3f const& dir, DirSampler::Info& si) const
{
	auto ctx = BuildPhongContext(sInfo, *this);
	EvaluatePhongBlinn(ctx, PhongType::Phong, sInfo, dir, si);
}


// Microfacet material
static inline float GGX_D(float alpha2, float NoH)
{
	float denom = NoH * NoH * (alpha2 - 1.0f) + 1.0f;
	return alpha2 / (float(M_PI) * denom * denom);
}

static inline float GGX_G1(float alpha2, float NoX)
{
	float denom = NoX + std::sqrt(alpha2 + (1.0f - alpha2) * NoX * NoX);
	return (2.0f * NoX) / denom;
}

static inline float GGX_G(float alpha2, float NoL, float NoV)
{
	return GGX_G1(alpha2, NoL) * GGX_G1(alpha2, NoV);
}

static inline Color FresnelSchlick(Color const& F0, float VoH)
{
	float k = std::pow(1.0f - VoH, 5.0f);
	return F0 + (Color(1.0f) - F0) * k;
}

// Microfacet material
bool MtlMicrofacet::GenerateSample(SamplerInfo const& sInfo, Vec3f& dir, DirSampler::Info& si) const
{
	si.SetVoid();
	Vec3f N = sInfo.N(); N.Normalize();
	if (!sInfo.IsFront()) N = -N;
	Vec3f V = SafeNormalize(sInfo.V());

	Color base = sInfo.Eval(baseColor);
	float rough = Clamp01(sInfo.Eval(roughness));
	float metal = Clamp01(sInfo.Eval(metallic));
	Color trans = sInfo.Eval(transmittance);
	float alpha = max(0.002f, rough * rough);

	Color F0 = Color(0.04f) * (1.0f - metal) + base * metal;
	Color diffuse = base * (1.0f - metal);

	float pd = diffuse.Luma1();
	float ps = F0.Luma1();
	float pt = trans.Luma1();
	float total = pd + ps + pt;
	if (total <= 0.0f) return false;

	float r = sInfo.RandomFloat() * total;
	if (r < pd) {
		Vec3f U, Vb; BuildONB(N, U, Vb);
		float u1 = sInfo.RandomFloat();
		float u2 = sInfo.RandomFloat();
		float rxy = std::sqrt(u1);
		float phi = 2.0f * float(M_PI) * u2;
		float x = rxy * std::cos(phi);
		float y = rxy * std::sin(phi);
		float z = std::sqrt(max(0.0f, 1.0f - u1));
		dir = SafeNormalize(U * x + Vb * y + N * z);

		float cosN = max(0.0f, N % dir);
		float pdf = cosN / float(M_PI);
		si.mult = diffuse * (1.0f / float(M_PI)) * cosN;
		si.prob = (pd / total) * pdf;
		si.dist = 0.0f;
		si.lobe = DirSampler::Lobe::DIFFUSE;
		return si.prob > 0.0f && si.mult.Max() > 0.0f;
	}
	else if (r < pd + ps) {
		Vec3f U, Vb; BuildONB(N, U, Vb);
		float u1 = sInfo.RandomFloat();
		float u2 = sInfo.RandomFloat();
		float phi = 2.0f * float(M_PI) * u1;
		float cosT = std::sqrt((1.0f - u2) / (1.0f + (alpha * alpha - 1.0f) * u2));
		float sinT = std::sqrt(max(0.0f, 1.0f - cosT * cosT));
		Vec3f h = SafeNormalize(U * (std::cos(phi) * sinT) + Vb * (std::sin(phi) * sinT) + N * cosT);
		dir = Reflect(-V, h);
		if ((N % dir) <= 0.0f) return false;

		float NoV = max(0.0f, N % V);
		float NoL = max(0.0f, N % dir);
		float NoH = max(0.0f, N % h);
		float VoH = max(0.0f, V % h);

		float alpha2 = alpha * alpha;
		float D = GGX_D(alpha2, NoH);
		float G = GGX_G(alpha2, NoL, NoV);
		Color F = FresnelSchlick(F0, VoH);

		// si.mult is the BRDF multiplied by the cosine term (NoL)
		// The unstable f = D*G*F / (4*NoV*NoL) is avoided by directly
		// computing f*NoL = D*G*F / (4*NoV), which is more stable
		si.mult = (D * G * F) / max(1e-6f, 4.0f * NoV);
		float pdfDir = (D * NoH) / max(1e-6f, 4.0f * VoH);

		si.prob = (ps / total) * pdfDir;
		si.dist = 0.0f;
		si.lobe = DirSampler::Lobe::SPECULAR;
		return si.prob > 0.0f && si.mult.Max() > 0.0f;
	}
	else {
		float eta = sInfo.IsFront() ? 1.0f / IOR() : IOR();
		Vec3f I = -V;
		float cosI = Clamp01(-(N % I)); // == Clamp01(N % V)
		float k = 1.0f - eta * eta * (1.0f - cosI * cosI);
		if (k < 0.0f) return false;
		Vec3f T = eta * I + (eta * cosI - std::sqrt(k)) * N;
		dir = SafeNormalize(T);
		float cosN = std::fabs(N % dir);

		float etaScale = TransmissionScale(eta, cosI, cosN);
		si.mult = trans * etaScale;
		si.prob = pt / total;
		si.dist = -1.0f; // use negative distance to flag delta transmission
		si.lobe = DirSampler::Lobe::TRANSMISSION;
		return si.prob > 0.0f && si.mult.Max() > 0.0f;
	}
}

void MtlMicrofacet::GetSampleInfo(SamplerInfo const& sInfo, Vec3f const& dirIn, DirSampler::Info& si) const
{
	si.SetVoid();
	Vec3f N = sInfo.N(); N.Normalize();
	if (!sInfo.IsFront()) N = -N;
	Vec3f V = SafeNormalize(sInfo.V());
	Vec3f L = SafeNormalize(dirIn);

	Color base = sInfo.Eval(baseColor);
	float rough = Clamp01(sInfo.Eval(roughness));
	float metal = Clamp01(sInfo.Eval(metallic));
	Color trans = sInfo.Eval(transmittance);
	float alpha = max(0.002f, rough * rough);

	Color F0 = Color(0.04f) * (1.0f - metal) + base * metal;
	Color diffuse = base * (1.0f - metal);

	float pd = diffuse.Luma1();
	float ps = F0.Luma1();
	float pt = trans.Luma1();
	float total = pd + ps + pt;
	if (total <= 0.0f) return;

	float NoL = N % L;
	Color mult(0.0f);
	float pdf = 0.0f;

	if (NoL > 0.0f) {
		// Diffuse contribution
		Color fDiff_term = diffuse * (1.0f / float(M_PI)) * NoL;
		float pdfDiff = (pd > 0.0f) ? (NoL / float(M_PI)) * (pd / total) : 0.0f;

		// Specular contribution
		Vec3f H = SafeNormalize(L + V);
		float NoV = max(0.0f, N % V);
		float NoH = max(0.0f, N % H);
		float VoH = max(0.0f, V % H);

		float pdfSpec = 0.0f;
		Color fSpec_term(0.0f); // This is BRDF * NoL
		if (ps > 0.0f && VoH > 0.0f) {
			float alpha2 = alpha * alpha;
			float D = GGX_D(alpha2, NoH);
			float G = GGX_G(alpha2, NoL, NoV);
			Color F = FresnelSchlick(F0, VoH);
			// Use stable formulation for f*NoL = D*G*F / (4*NoV)
			fSpec_term = (D * G * F) / max(1e-6f, 4.0f * NoV);
			pdfSpec = (D * NoH) / max(1e-6f, 4.0f * VoH);
			pdfSpec *= (ps / total);
		}

		mult = fDiff_term + fSpec_term;
		pdf = pdfDiff + pdfSpec;
		si.lobe = (ps > 0.0f && pd > 0.0f) ? DirSampler::Lobe::ALL : (ps > 0.0f ? DirSampler::Lobe::SPECULAR : DirSampler::Lobe::DIFFUSE);
	}
	else {
		if (pt > 0.0f) {
			float eta = sInfo.IsFront() ? 1.0f / IOR() : IOR();
			Vec3f I = -V;
			float cosI = -(N % I);
			float k = 1.0f - eta * eta * (1.0f - cosI * cosI);
			if (k < 0.0f) return;
			float cosT = std::sqrt(k);
			Vec3f T = SafeNormalize(eta * I + (eta * cosI - cosT) * N);
			// Only count the delta refraction direction
			if ((T % L) < 1.0f - 1e-4f) return;

			float etaScale = TransmissionScale(eta, cosI, cosT);
			mult = trans * etaScale;
			pdf = pt / total;
			si.lobe = DirSampler::Lobe::TRANSMISSION;
		}
	}

	if (pdf <= 0.0f || mult.Max() <= 0.0f) { si.SetVoid(); return; }
	si.mult = mult;
	si.prob = pdf;
	si.dist = 0.0f;
}

#endif // !SHADING_USE_LEGACY
