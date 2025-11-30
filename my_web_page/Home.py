import streamlit as st
import os
from PIL import Image, ImageOps, ImageDraw

st.set_page_config(page_title="Welcome", page_icon="👋")

st.title("Welcome to my web page!")

# --- Avatar + Intro in 2 columns ---
col1, col2 = st.columns([1, 3])

BASE_DIR = os.path.dirname(__file__)  # directory where Home.py is
IMAGE_PATH = os.path.join(BASE_DIR, "assets/avatar2.png")

with col1:
    if os.path.exists(IMAGE_PATH):
        # Open image
        img = Image.open(IMAGE_PATH)
        # Resize to 150x150 with high-quality resampling
        img = img.resize((150, 150), Image.LANCZOS)
        # Make circular
        mask = Image.new("L", (150, 150), 0)
        draw = ImageDraw.Draw(mask)
        draw.ellipse((0, 0, 150, 150), fill=255)
        img.putalpha(mask)
        st.image(img)
    else:
        st.warning(f"Avatar image not found at {IMAGE_PATH}")

with col2:
    st.markdown("""
    ### 👋 Hello!

    I am **Urša Kumelj**, a **second-year master’s student** in the program  
    **Computer science and mathematics** at the  
    **Faculty of Mathematics and Physics (FMF)**.

    I currently work as a student at  
    **Institut Jožef Stefan**, **Department E7**.

    This website presents the work I’ve done on  
    **Evolutionary Algorithms** and **Optimization**.
    """)

st.markdown("---")

st.subheader("📌 Navigation")
st.info("If the page fails to load properly, try refreshing it.")
st.write("Use the sidebar or the buttons below to explore the pages.")

# Buttons to navigate to other pages
if st.button(":material/insights: Evolutionary algorithm benchmarking tool"):
    st.switch_page("pages/1_Benchmarking.py")

if st.button(":material/directions_car: Vehicle routing problem solver with OR-Tools"):
    st.switch_page("pages/2_Ortools.py")

if st.button(":material/auto_awesome: Vehicle routing problem solver with Nevergrad"):
    st.switch_page("pages/3_Nevergrad.py")