---
layout: post
title: "Build a Simple CRUD API with Python, Flask, and MongoDB in Docker"
categories: [tutorial, python, flask, mongodb, docker]
tags: [python, flask, mongodb, docker, api, tutorial]
---

## Introduction

In this tutorial, I'll guide you through building a simple CRUD (Create, Read, Update, Delete) API using Python and Flask, with MongoDB as our database. To keep things clean and manageable, we'll run MongoDB in a Docker container, so you don't need to install it locally. This approach provides a great way to isolate your development environment and avoid cluttering your system.

**Note:** This will not create persistent storage allowing you to use this as a playground. For persistent storage, you could visit the repository mentioned in "Further Reading" at the end of the article.

## Prerequisites

- Basic understanding of Python and Flask.
- Docker installed on your system.
- Python installed on your system.

## Step 1: Setting Up the Development Environment

### 1. Create a Virtual Environment (Recommended):

```bash
python3 -m venv .venv
source .venv/bin/activate  # On Windows, use .venv\Scripts\activate
```

### 2. Install Required Python Packages:

```bash
pip install Flask pymongo
```

## Step 2: Running MongoDB in a Docker Container

Instead of installing MongoDB locally, we'll use Docker to run MongoDB in a container.

### 1. Pull the MongoDB Docker Image:

```bash
docker pull mongo
```

### 2. Run MongoDB in a Container:

```bash
docker run --rm --name mongodb-server -d -p 27017:27017 mongo
```

This command starts a MongoDB server in a Docker container, mapping its default port (27017) to your local machine.

## Step 3: Creating the Flask App

Create a new Python file called `app.py` and set up a basic Flask app connected to MongoDB.

```python
from flask import Flask, jsonify, request
from pymongo import MongoClient

app = Flask(__name__)
client = MongoClient("mongodb://localhost:27017/")
db = client.test_db  # Database name

@app.route('/')
def index():
    return 'Welcome to the CRUD API!'
```

## Step 4: Defining CRUD Operations

We'll add routes to handle basic CRUD operations for products and users.

### Create Operations (POST):

```python
@app.route('/products', methods=['POST'])
def add_product():
    product_data = request.get_json()
    result = db.products.insert_one(product_data)
    return jsonify({"message": "Product created successfully", "id": str(result.inserted_id)}), 201

@app.route('/users', methods=['POST'])
def add_user():
    user_data = request.get_json()
    result = db.users.insert_one(user_data)
    return jsonify({"message": "User created successfully", "id": str(result.inserted_id)}), 201
```

### Read Operations (GET):

```python
@app.route('/products/<product_id>', methods=['GET'])
def get_product(product_id):
    product = db.products.find_one({"product_id": product_id})
    if product:
        product['_id'] = str(product['_id'])
        return jsonify(product)
    else:
        return jsonify({"message": "Product not found"}), 404

@app.route('/users/<username>', methods=['GET'])
def get_user(username):
    user = db.users.find_one({"username": username})
    if user:
        user['_id'] = str(user['_id'])
        return jsonify(user)
    else:
        return jsonify({"message": "User not found"}), 404
```

### Update Operations (PUT):

```python
@app.route('/products/<product_id>', methods=['PUT'])
def update_product(product_id):
    update_data = request.get_json()
    result = db.products.update_one({"product_id": product_id}, {"$set": update_data})
    if result.matched_count:
        return jsonify({"message": "Product updated successfully"}), 200
    else:
        return jsonify({"message": "Product not found"}), 404

@app.route('/users/<username>', methods=['PUT'])
def update_user(username):
    update_data = request.get_json()
    result = db.users.update_one({"username": username}, {"$set": update_data})
    if result.matched_count:
        return jsonify({"message": "User updated successfully"}), 200
    else:
        return jsonify({"message": "User not found"}), 404
```

### Delete Operations (DELETE):

```python
@app.route('/products/<product_id>', methods=['DELETE'])
def delete_product(product_id):
    result = db.products.delete_one({"product_id": product_id})
    if result.deleted_count:
        return jsonify({"message": "Product deleted successfully"}), 200
    else:
        return jsonify({"message": "Product not found"}), 404

@app.route('/users/<username>', methods=['DELETE'])
def delete_user(username):
    result = db.users.delete_one({"username": username})
    if result.deleted_count:
        return jsonify({"message": "User deleted successfully"}), 200
    else:
        return jsonify({"message": "User not found"}), 404
```

### Add Debug Configuration for Testing:

```python
if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
```

## Step 5: Running the Flask App

Run your Flask app with the command:

```bash
python app.py
```

## Step 6: Testing the API

You can test your API using tools like Postman or curl. Here's an example of how to add a product using curl:

```bash
curl -X POST http://localhost:5000/products \
-H "Content-Type: application/json" \
-d '{"product_id": "123", "name": "Example Product", "price": 29.99}'
```

If everything went fine, you should see a JSON string returned in the console, for example:

```json
{
  "id": "66c2fc1fc6b4866cfba83652",
  "message": "Product created successfully"
}
```

Now you can try with the other URLs. For example:

```bash
curl -X GET http://localhost:5000/products/123
```

Should produce the following output:

```json
{
  "_id": "66c2fc1fc6b4866cfba83652",
  "name": "Example Product",
  "price": 29.99,
  "product_id": "123"
}
```

## Takeaway

By following these steps, you've set up a basic CRUD API using Flask and MongoDB, with MongoDB running inside a Docker container. This setup keeps your development environment clean and ensures that MongoDB doesn't interfere with other projects on your system. As you continue to build out your application, you can easily expand this foundation to include more features and handle more complex data.

## Further Reading

If you are interested, I have a more detailed guide on my repository:

- **Using Docker Networks**
- **Docker Compose**
- **Testing with Postman**

[**GitHub - wambitz/mongo-python-api**](https://github.com/wambitz/mongo-python-api): In this repository, there are a few examples on how to use MongoDB with the Python Client.