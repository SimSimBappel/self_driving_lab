# Supabase Docker

# Start the services (in detached mode)
```
# Start in detached mode
docker compose up -d

# Check running
docker compose ps

# Connect to interface
url:      localhost:8000
username: supabase
password: this_password_is_insecure_and_should_be_updated
```


# Other commands
```
# Stop docker and remove volumes:
docker compose down -v

# Format system (Use with caution)
rm -rf volumes/db/data/
```