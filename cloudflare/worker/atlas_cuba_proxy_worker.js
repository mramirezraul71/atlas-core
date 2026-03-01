export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const target = url.searchParams.get("url");
    if (!target) {
      return new Response("Missing ?url=", { status: 400 });
    }
    const parsed = new URL(target);
    const host = parsed.hostname.toLowerCase();
    const allowed = (env.ALLOWED_HOSTS || "").split(",").map(s => s.trim().toLowerCase()).filter(Boolean);
    if (allowed.length && !allowed.includes(host)) {
      return new Response("Host not allowed", { status: 403 });
    }
    const upstreamReq = new Request(target, {
      method: request.method,
      headers: request.headers,
      body: request.body,
      redirect: "follow",
    });
    const res = await fetch(upstreamReq, { cf: { cacheEverything: false } });
    const out = new Response(res.body, res);
    out.headers.set("x-atlas-worker", "atlas-cuba-proxy");
    return out;
  }
};
