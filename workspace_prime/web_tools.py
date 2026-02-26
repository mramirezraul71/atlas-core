import httpx


class WebTools:
    def __init__(self, timeout: int = 30):
        self.timeout = timeout
        self.headers = {"User-Agent": "ATLAS-WORKSPACE-PRIME/3.0"}

    def fetch(
        self,
        url: str,
        method: str = "GET",
        body: dict = None,
        extra_headers: dict = None,
    ) -> dict:
        try:
            h = {**self.headers, **(extra_headers or {})}
            with httpx.Client(timeout=self.timeout) as client:
                if method.upper() == "POST":
                    r = client.post(url, json=body, headers=h)
                else:
                    r = client.get(url, headers=h)
            return {
                "success": True,
                "status_code": r.status_code,
                "content": r.text[:5000],
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def call_api(
        self, url: str, api_key: str = None, method: str = "GET", body: dict = None
    ) -> dict:
        headers = {"Authorization": f"Bearer {api_key}"} if api_key else {}
        return self.fetch(url, method, body, headers)

    def search_ddg(self, query: str) -> dict:
        url = f"https://api.duckduckgo.com/?q={query}&format=json&no_html=1"
        return self.fetch(url)


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    wt = WebTools()
    r = wt.fetch("https://httpbin.org/get")
    print("OK WebTools - status:", r.get("status_code"))
