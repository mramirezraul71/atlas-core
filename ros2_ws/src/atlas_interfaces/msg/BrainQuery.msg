# Query/response for brain bridge (AI reasoning)
string query_id
string query_text                    # natural language query to AI
string model_hint                    # preferred model (e.g. "bedrock:opus-4.6")
string context_json                  # serialized context (sensor state, task history)
string response_text                 # filled by brain bridge on response
bool success
float64 latency_ms
