<div align="center">

# 🧩 KiCad PCB Design  
### Design of Transmitter for the Flight Controller using KiCad

<img src="media/kicad_logo.png" alt="KiCad Logo" width="180"/>

</div>

---

## 🛠️ To Design a Custom Symbol

1. A **symbol** is a representation of a component.  
2. It doesn’t reflect the physical **dimensions** of the component.  
3. It should have **all pin names** clearly labeled.

---

### ⚙️ Steps to Create a Custom Symbol in KiCad

1. Open **KiCad**.  
2. Open **Symbol Editor**.  

<p align="center">
  <img src="media/Screenshot1.png" alt="KiCad Symbol Editor" width="400"/>
</p>

3. Choose **File → New Library**.  

<p align="center">
  <img src="media/Screenshot2.png" alt="New Library" width="400"/>
</p>

4. Choose **Global** to use the symbol in every project,  
   or **Project** to use it only for the current project.  
   (Here, I chose *Global*.)

<p align="center">
  <img src="media/Screenshot3.png" alt="Choose Library Scope" width="400"/>
</p>

5. Save the new library in the **symbols directory** under your desired name.

6. Right-click on the library you made (**CustomLibrary**) and select **New Symbol**.  

<p align="center">
  <img src="media/Screenshot4.png" alt="New Symbol" width="400"/>
</p>

7. Edit the **Symbol Name** and click **OK**.  
   If you want to develop your symbol from an existing one,  
   check **Derive from existing symbol** and select the location.  

<p align="center">
  <img src="media/Screenshot5.png" alt="Edit Symbol Name" width="400"/>
</p>

8. Choose **Draw Rectangle** to create the symbol layout.  

<p align="center">
  <img src="media/Screenshot6.png" alt="Draw Rectangle" width="250"/>
</p>

9. Draw the symbol layout to include all pin names.  

<p align="center">
  <img src="media/Screenshot7.png" alt="Draw Symbol Layout" width="400"/>
</p>
