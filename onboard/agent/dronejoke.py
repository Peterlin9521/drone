import os
from portkey_ai import Portkey

portkey = Portkey(
  api_key = os.environ["LLM_API_KEY"]
)

response = portkey.chat.completions.create(
    model = "@zotgpt-api-azure/gpt-5.5",
    messages = [
      {"role": "system", "content": "You are a helpful assistant."},
      {"role": "user", "content": "Tell me a joke about drones."}
    ],
    MAX_TOKENS = 512
)

print(response.choices[0].message.content)

