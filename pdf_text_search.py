import os
import sys
import re
import tempfile
import webbrowser
import pathlib
from PyPDF2 import PdfReader
import subprocess

DEBUG = True  # Set to False when done testing

def write_results_to_tempfile(message):
    """Write message to a temporary HTML file and open it in the default browser."""
    # Create a unique temp file
    fd, temp_file = tempfile.mkstemp(suffix=".html", prefix="pdf_search_")
    os.close(fd)  # Close the low-level file descriptor

    # Wrap in <pre> so spacing is preserved
    html_content = f"<html><body><pre>{message}</pre></body></html>"

    with open(temp_file, "w", encoding="utf-8") as f:
        f.write(html_content)

    print(f"Results written to: {temp_file}")

    # Use pathlib to create a proper file URL
    file_url = pathlib.Path(temp_file).resolve().as_uri()

    # Open in default browser
    webbrowser.open(file_url)

    return temp_file

def find_pdf_with_number(search_dir, drawing_number):
    """Find the first PDF file that includes the drawing number in its filename."""
    for root, dirs, files in os.walk(search_dir):
        for file in files:
            if file.lower().endswith(".pdf") and drawing_number in file:
                return os.path.join(root, file)
    return None

def search_pdf_for_terms(pdf_path, search_terms):
    """Return list of found terms in the given PDF."""
    found = set()
    try:
        reader = PdfReader(pdf_path)
        for page in reader.pages:
            text = page.extract_text() or ""
            for term in search_terms:
                if term in text:  # simpler, more reliable match
                    found.add(term)
        return list(found)
    except Exception as e:
        return [f"Error reading {pdf_path}: {e}"]

def main():
    if len(sys.argv) < 3:
        print("Usage: python pdf_text_search_tempfile.py <search_dir> <drawing_number> [term1 term2 ...]")
        sys.exit(1)

    search_dir = sys.argv[1]
    drawing_number = sys.argv[2]
    search_terms = sys.argv[3:]

    message_lines = [f"Panel number: {drawing_number}", f"Search input: {search_terms}\n"]

    pdf_path = find_pdf_with_number(search_dir, drawing_number)
    if not pdf_path:
        message_lines.append(f"No PDF found containing '{drawing_number}'")
        write_results_to_tempfile("\n".join(message_lines))
        sys.exit(0)

    found_terms = search_pdf_for_terms(pdf_path, search_terms)

    if found_terms:
        message_lines.append(f"Found detail(s): {', '.join(found_terms)}")
        for i in found_terms:
            if i == "P-203":
                message_lines.append(f"Corner Brick detail. Need to cut 1in from edge of panel.")
        write_results_to_tempfile("\n".join(message_lines))
    
    else:
        message_lines.append("No important details found.")

if __name__ == "__main__":
    main()
