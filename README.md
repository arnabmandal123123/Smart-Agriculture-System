# Portfolio Website

A modern, responsive portfolio website to showcase your GitHub projects and skills.

## Features

- 🎨 Modern, clean design with dark theme
- 📱 Fully responsive (mobile, tablet, desktop)
- 🚀 Automatic GitHub project fetching via API
- 🏷️ Project filtering by category (IoT, Web, Embedded)
- ✨ Smooth animations and interactions
- 📊 Project statistics (stars, forks, language)
- 📧 Contact form
- 🔗 Social media links

## Setup Instructions

### 1. Update Configuration

Open `script.js` and update the `CONFIG` object with your information:

```javascript
const CONFIG = {
    githubUsername: 'your-github-username',  // Your GitHub username
    githubToken: '',                         // Optional: GitHub personal access token
    email: 'your.email@example.com',
    linkedin: 'your-linkedin-username',
    location: 'Your City, Country',
    name: 'Your Full Name'
};
```

### 2. Update Manual Projects (Optional)

If you want to showcase specific projects or the GitHub API fails, update the `manualProjects` array in `script.js`:

```javascript
manualProjects: [
    {
        name: 'Project Name',
        description: 'Project description',
        category: ['iot', 'embedded'],  // Categories: iot, web, embedded
        tags: ['Arduino', 'ESP32'],
        html_url: 'https://github.com/username/project',
        stargazers_count: 10,
        forks_count: 2,
        language: 'C++',
        topics: []
    }
]
```

### 3. GitHub API Rate Limits

The GitHub API allows 60 requests per hour without authentication. For unlimited access:

1. Go to GitHub Settings → Developer Settings → Personal Access Tokens
2. Generate a new token (select `public_repo` scope)
3. Add the token to `CONFIG.githubToken` in `script.js`

### 4. Customize Content

Edit `index.html` to update:
- Hero section text and description
- About section content
- Skills and technologies
- Contact information

### 5. Deploy

You can deploy this portfolio to:

- **GitHub Pages**: 
  1. Create a repository named `username.github.io`
  2. Push these files to the repository
  3. Access at `https://username.github.io`

- **Netlify/Vercel**: 
  1. Connect your GitHub repository
  2. Deploy automatically

- **Local Server**: 
  ```bash
  # Using Python
  python -m http.server 8000
  
  # Or using VS Code Live Server extension
  ```

## File Structure

```
portfolio/
├── index.html          # Main HTML file
├── style.css           # All styling
├── script.js           # JavaScript functionality
└── README.md          # Documentation
```

## Customization Tips

### Colors
Update CSS variables in `style.css`:
```css
:root {
    --primary-color: #6366f1;
    --secondary-color: #8b5cf6;
    --accent-color: #ec4899;
}
```

### Fonts
Add custom fonts in the `<head>` of `index.html`:
```html
<link href="https://fonts.googleapis.com/css2?family=Your+Font&display=swap" rel="stylesheet">
```

### Contact Form
Integrate with services like:
- [Formspree](https://formspree.io/)
- [EmailJS](https://www.emailjs.com/)
- [Netlify Forms](https://www.netlify.com/products/forms/)

## Browser Support

- Chrome (latest)
- Firefox (latest)
- Safari (latest)
- Edge (latest)

## License

Feel free to use this template for your personal portfolio!

## Credits

Built with HTML, CSS, and JavaScript
Icons from Font Awesome

---

Made with ❤️ for showcasing amazing projects!
