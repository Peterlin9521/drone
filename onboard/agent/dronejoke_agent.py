import os
import re
import sys
 
from portkey_ai import Portkey
 
# Put your key in an environment variable (recommended):
#   export PORTKEY_API_KEY="your-key"
# or paste it below. Never upload a file with a real key to Canvas/GitHub.
API_KEY = os.environ.get("PORTKEY_API_KEY", "REMOVED")
MODEL = "@zotgpt-api-azure/gpt-5.5"
 
if API_KEY.startswith("PUT-YOUR"):
    sys.exit("Set PORTKEY_API_KEY or edit API_KEY at the top of this file.")
 
portkey = Portkey(api_key=API_KEY)
 
 
def ask(system_msg, user_msg, budget=1024):
    """Call the LLM and return its text. The budget is large because this
    model spends tokens on internal reasoning before the visible answer;
    an empty reply with finish_reason='length' means it ran out."""
    response = portkey.chat.completions.create(
        model=MODEL,
        messages=[
            {"role": "system", "content": system_msg},
            {"role": "user", "content": user_msg},
        ],
        max_completion_tokens=budget,  # NOT max_tokens: this model rejects it
    )
    choice = response.choices[0]
    text = (choice.message.content or "").strip()
    if not text:
        raise SystemExit(
            f"Empty reply (finish_reason={choice.finish_reason!r}). "
            "If 'length', raise the token budget."
        )
    return text
 
 
# Step 1: get a joke
joke = ask("You are a helpful assistant.", "Tell me a joke about drones.")
print("Joke:", joke)
 
# Step 2: send the joke back and ask for a yes/no judgement
verdict = ask(
    "You are a strict comedy critic. Reply with exactly one word: YES or NO.",
    f"Is this joke funny?\n\n{joke}",
)
print("LLM verdict:", verdict)
 
# Step 3: act on the answer (Python checks it, never assumes)
answer = verdict.upper()
if re.search(r"\bYES\b", answer) and not re.search(r"\bNO\b", answer):
    print("haha")
elif re.search(r"\bNO\b", answer):
    print("not funny")
else:
    print(f"Could not read a yes/no answer: {verdict!r}")
 
