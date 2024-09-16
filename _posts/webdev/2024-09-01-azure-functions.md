---
layout: post
title:  "Create and Run Python-based Azure Functions Locally: A Step-by-Step Guide"
categories: Azure, Python, Development
---

## Getting Started

This small project demonstrates how to create, develop, and test a Python-based Azure Function locally using the Azure Functions Core Tools, the VS Code editor with the Azure Functions extension, and the command line. It includes a simple HTTP trigger function that responds to GET and POST requests.

These instructions will get your copy of the project up and running on your local machine for development and testing purposes, using both CLI and VS Code.

## Prerequisites

- Python (3.10+ recommended)
- Azure Functions Core Tools
- [Visual Studio Code](https://code.visualstudio.com/) (Optional but recommended)
- [Azure Functions Extension for VS Code](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-azurefunctions)
- Any web browser or `curl` (for testing)

### Step 1: Install Python

Ensure Python 3.10 or later is installed on your system. You can download Python from the [official Python website](https://www.python.org/downloads/).

### Step 2: Install Azure Functions Core Tools

Follow the instructions provided on the [official Microsoft documentation](https://learn.microsoft.com/en-us/azure/azure-functions/functions-run-local?tabs=windows%2Cisolated-process%2Cnode-v4%2Cpython-v2%2Chttp-trigger%2Ccontainer-apps&pivots=programming-language-python) to install the Azure Functions Core Tools.

### Step 3: VS Code Setup (Optional)

Using Visual Studio Code (VS Code) for developing Azure Functions can enhance your workflow with built-in debugging, IntelliSense, and more.

1. **Install VS Code**: If you haven’t installed it yet, download it from the [VS Code website](https://code.visualstudio.com/).
   
2. **Install the Azure Functions Extension**: Open VS Code and install the Azure Functions extension from the Marketplace. You can also install it via the command palette by searching for "Azure Functions" or directly from [here](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-azurefunctions).
   
3. **Install the Python Extension**: Install the [Python extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python) to enable IntelliSense and Python debugging support.

4. **Install Azure CLI** (optional but useful for Azure deployments): Download and install the [Azure CLI](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli).

### Step 4: Create Your Project Directory

Create your project directory using the command line:

```powershell
mkdir azure_functions
cd azure_functions
```

Alternatively, you can create and open this directory from within VS Code by clicking on **File > Open Folder** and selecting the `azure_functions` directory.

### Step 5: Set Up Your Virtual Environment (Recommended)

Start your virtual environment from the terminal:

```powershell
py -3.10 -m venv .venv
.venv\Scripts\activate  # source .venv/bin/activate for Linux
```

### Step 6: Initialize the Project

#### Option 1: Using the Command Line

Use the Azure Functions Core Tools to initialize the project:

```powershell
func init
```

You'll be prompted to select a language. Pick **Python**:

```powershell
Use the up/down arrow keys to select a worker runtime:
dotnet
dotnet (isolated process)
node
python # < Select this one
powershell
custom
```

#### Option 2: Using VS Code

1. Open your project folder in VS Code.
2. Press **Ctrl + Shift + P** (or **Cmd + Shift + P** on Mac) to open the command palette and type `Azure Functions: Create New Project...`.
3. Select the folder where you want to create the project.
4. When prompted for the runtime, select **Python** and choose the appropriate version (e.g., Python 3.10).
5. VS Code will scaffold the project for you.

This is how your project structure should look:

```powershell
Mode                 LastWriteTime         Length Name
----                 -------------         ------ ----
d----           9/15/2024  7:50 PM                .venv
d----           9/15/2024  7:52 PM                .vscode
-a---           9/15/2024  7:52 PM              2 .funcignore
-a---           9/15/2024  7:52 PM            517 .gitignore
-a---           9/15/2024  7:52 PM            104 function_app.py
-a---           9/15/2024  7:52 PM            302 host.json
-a---           9/15/2024  7:52 PM            206 local.settings.json
-a---           9/15/2024  7:52 PM            207 requirements.txt
```

### Step 7: Install Requirements

Install the generated requirements:

```powershell
pip install -r requirements.txt
```

Output:

```powershell
Collecting azure-functions
  Using cached azure_functions-1.20.0-py3-none-any.whl (181 kB)
Installing collected packages: azure-functions
Successfully installed azure-functions-1.20.0
```

### Step 8: Create a New Function

#### Option 1: Using the Command Line

To create a new function, use the following command:

```powershell
func new
```

Select `HTTP Trigger` and give it a name (default: `http_trigger`).

```powershell
Use the up/down arrow keys to select a template:
...
HTTP trigger # < Select this one
...
```

Press enter to use the default name:

```powershell
Use the up/down arrow keys to select a template:Function Name: [http_trigger]
```

For `AUTH_LEVEL` select `ANONYMOUS`:

```powershell
Use the up/down arrow keys to select a Auth Level:
FUNCTION
ANONYMOUS # < Select this one
ADMIN
```

#### Option 2: Using VS Code

1. In the **Azure** panel on the left sidebar of VS Code, click on **Create Function** (the lightning bolt icon).
2. Select **HTTP Trigger** as the function template.
3. Give the function a name (e.g., `http_trigger`).
4. Set the authorization level to **ANONYMOUS**.

VS Code will automatically create the new function and update the `function_app.py` file with the following code:

```python
import azure.functions as func
import datetime
import json
import logging

app = func.FunctionApp()

@app.route(route="http_trigger", auth_level=func.AuthLevel.ANONYMOUS)
def http_trigger(req: func.HttpRequest) -> func.HttpResponse:
    logging.info('Python HTTP trigger function processed a request.')

    name = req.params.get('name')
    if not name:
        try:
            req_body = req.get_json()
        except ValueError:
            pass
        else:
            name = req_body.get('name')

    if name:
        return func.HttpResponse(f"Hello, {name}. This HTTP triggered function executed successfully.")
    else:
        return func.HttpResponse(
             "This HTTP triggered function executed successfully. Pass a name in the query string or in the request body for a personalized response.",
             status_code=200
        )
```

### Step 9: Start Your Local Test Server

#### Option 1: Using the Command Line

Start the app with:

```powershell
func start
```

This will display the URL for you to test on `localhost`:

```
Found Python version 3.10.11 (py).

Azure Functions Core Tools
Core Tools Version:       4.0.5611 Commit hash: N/A +591b8aec842e333a87ea9e23ba390bb5effe0655 (64-bit)
Function Runtime Version: 4.31.1.22191

[2024-09-16T04:26:12.335Z] Worker process started and initialized.

Functions:

        http_trigger:  http://localhost:7071/api/http_trigger

For detailed output, run func with --verbose flag.
[2024-09-16T04:26:17.769Z] Executing 'Functions.http_trigger' (Reason='This function was programmatically called via the host APIs.', Id=6cd3e835-3ff0-44e8-9b63-819c8a21cd20)
[2024-09-16T04:26:17.819Z] Python HTTP trigger function processed a request.
[2024-09-16T04:26:17.938Z] Executed 'Functions.http_trigger' (Succeeded, Id=6cd3e835-3ff0-44e8-9b63-819c8a21cd20, Duration=184ms)
```

#### Option 2: Using VS Code

1. Press **F5** in VS Code to start the function.
2. The integrated terminal will display the URL for your function:

```
http://localhost:7071/api/http_trigger
```

### Step 10: Test Your App Locally

Use a web browser or `curl` to test the function:

```
http://localhost:7071/api/http_trigger
```

Alternatively test using `curl` from `WSL` or `Powershell` (if available):

```bash
  curl http://localhost:7071/api/http_trigger
```

Output:

```
This HTTP triggered function executed successfully. Pass a name in the query string or in the request body for a personalized response.
```

Try it now passing a name argument!

```bash
curl http://localhost:7071/api/http_trigger?name=JohnDoe
```

Output:

```
Hello, JohnDoe. This HTTP triggered function executed successfully.
```

## Summary

Congratulations! You've successfully developed a Python-based Azure Function on your local machine using both the command line and VS Code. You've learned how to install Python and the Azure Functions Core Tools, set up a virtual environment, create and configure a new Azure Functions project, and write a simple HTTP trigger function. You've also explored how VS Code can streamline your development process with built-in debugging and project management tools.

Your function app is now ready to deploy on the Azure Cloud, where it can scale and be used as a serverless application. For deployment, you can follow the official documentation on [publishing Azure Functions](https://learn.microsoft.com/en-us/azure/azure-functions/functions-develop-local#publish-the-project-to-azure).
